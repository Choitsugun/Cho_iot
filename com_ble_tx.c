#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "config.h"
#include "app_error.h"
#include "ble_db_discovery.h"
#include "app_timer.h"
#include "app_trace.h"
#include "app_util.h"
#include "app_error.h"
#include "boards.h"
#include "ble.h"
#include "ble_gap.h"
#include "dual_mode.h"
#include "app_fifo.h"

app_fifo_t_ble  m_ble_tx_buffer;

static ble_tx ble_cmd_tx_buf[32];

ble_tx cmd_ble_tx_buffer[1];

uint8_t cmd_ble_string_p_buf[32][20];

/**
* @brief �f�L���[������
* @return	����
* @pre	����
* @post	����
*/

uint32_t cmd_ble_buffer_config()
{
	m_ble_tx_buffer.ble_tx_buffer   = ble_cmd_tx_buf; 

	for(int i = 0; i <32; i++)
	{
		ble_cmd_tx_buf[i].p_string = cmd_ble_string_p_buf[i];
	}
	
  m_ble_tx_buffer.buf_size_mask = 32 - 1;
  m_ble_tx_buffer.read_pos      = 0;
  m_ble_tx_buffer.write_pos     = 0;
}

/**
* @brief �f�L���[����
* @param[in]	p_buffer�@�L���[�|�C���^�[
* @param[in]	params�@���M�p�P�b�g��conhandle�Ȃ�
* @param[in]	p_string�@�t���[���|�C���^�[
* @param[in]	length�@�@�t���[������
* @return	����
* @pre	����
* @post	����
*/

static __INLINE void ble_buffer_put(app_fifo_t_ble* p_buffer, ble_nus_c_t* params, uint8_t* p_string, uint16_t length)
{
	p_buffer->ble_tx_buffer[p_buffer->write_pos & p_buffer->buf_size_mask].params = params;
	
	for( int i = 0; i < length; i++)
	{
		p_buffer->ble_tx_buffer[p_buffer->write_pos & p_buffer->buf_size_mask].p_string[i] = p_string[i];
	}
	
	p_buffer->ble_tx_buffer[p_buffer->write_pos & p_buffer->buf_size_mask].length = length;
	p_buffer->write_pos++;
}

/**
* @brief BLE �L���[���Ă��Ȃ����𔻒f
* @param[in]	p_buffer�@�����O�o�b�t�@�|�W�V�����t���O������\����
* @return	�@�@���[�h�ƃ��C�g�̃|�W�V�����ʒu��
* @pre	����
* @post	����
*/

static __INLINE uint32_t ble_buffer_length(app_fifo_t_ble* p_buffer)
{
	uint32_t tmp = p_buffer->read_pos;
  return p_buffer->write_pos - tmp;
}

uint32_t cmd_ble_buffer_put(app_fifo_t_ble* p_buffer, ble_nus_c_t* params, uint8_t* p_string, uint16_t length)
{
	if (ble_buffer_length(p_buffer) <= p_buffer->buf_size_mask)
  {   
		ble_buffer_put(p_buffer, params, p_string, length);
    return NRF_SUCCESS;
  }
  return NRF_ERROR_NO_MEM;
}

uint32_t cmd_ble_buffer(ble_nus_c_t* params, uint8_t* p_string, uint16_t length)
{
	uint32_t err_code;
	
	err_code = cmd_ble_buffer_put(&m_ble_tx_buffer, params, p_string, length);
	return err_code;
}

/**
* @brief BLE���M����
* @param[in]	params�@���M�p�P�b�g��conhandle�Ȃ�
* @param[in]	p_string�@�t���[���|�C���^�[
* @param[in]	length�@�@�t���[������
* @return	NRF_SUCCESS�F�ǉ����� NRF_ERROR_NO_MEM�F�f�[�^���
* @pre	����
* @post	����
*/

uint32_t cmd_ble_tx(ble_nus_c_t* params, uint8_t* p_string, uint16_t length)
{
	uint32_t err_code;
	
	err_code = ble_nus_c_string_send(params, p_string, length);
	
	switch(err_code)
	{
		case NRF_SUCCESS:
			return 0;
			
		case BLE_ERROR_INVALID_CONN_HANDLE:
			return 1;
			
		case NRF_ERROR_INVALID_STATE:
			err_code = cmd_ble_buffer(params, p_string, length);	  
			return 2;
			
		case NRF_ERROR_INVALID_PARAM:
			return 3;
		
		case BLE_ERROR_NO_TX_PACKETS:
			err_code = cmd_ble_buffer(params, p_string, length);
			return err_code;
		
		 default:
			return 5;	
	 }		 
}

/**
* @brief �L���[����f�[�^�����o��
* @param[in]	p_buffer�@�L���[�|�C���^�[
* @param[in]	p_byte�@�@�f�[�^�̃R�s�[��̃|�C���^�[
* @return	����
* @pre	����
* @post	����
*/

static __INLINE void ble_buffer_peek(app_fifo_t_ble* p_buffer, ble_tx* p_byte)
{
	p_byte->params = p_buffer->ble_tx_buffer[(p_buffer->read_pos) & p_buffer->buf_size_mask].params;
	p_byte->p_string = p_buffer->ble_tx_buffer[(p_buffer->read_pos) & p_buffer->buf_size_mask].p_string;
	p_byte->length = p_buffer->ble_tx_buffer[(p_buffer->read_pos) & p_buffer->buf_size_mask].length;
}

static __INLINE void ble_buffer_get(app_fifo_t_ble* p_buffer, ble_tx* p_byte)
{
	ble_buffer_peek(p_buffer, p_byte);
  p_buffer->read_pos++;
}

uint32_t cmd_ble_buffer_get(app_fifo_t_ble* p_buffer, ble_tx* p_byte)
{
	if (ble_buffer_length(p_buffer) != 0)
  {
		ble_buffer_get(p_buffer, p_byte);
    return NRF_SUCCESS;
  }
  return NRF_ERROR_NOT_FOUND;
}

uint32_t ble_buffer_softdevice_fifo(void)
{
	uint32_t err_code;
	
	err_code = cmd_ble_buffer_get(&m_ble_tx_buffer, cmd_ble_tx_buffer);
	return err_code;
}
