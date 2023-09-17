void USER_CAN1_transmit(uint8_t*TxData,uint8_t TxDatalen,int BOX,int can1_id,int IDE_mode);
void USER_CAN2_transmit(uint8_t*TxData,uint8_t TxDatalen,int BOX,int can2_id,int IDE_mode);
void USER_can_Init();
void CAN1_IdleCallback();
void CAN2_IdleCallback();

