#ifndef ENCODER_H_
#define ENCODER_H_

//#define INT_ENCODER_READ

#define EINT0		 0x00000001
#define EINT1		 0x00000002
#define EINT2		 0x00000004
#define EINT3        0x00000008

#define EINT0_EDGE   0x00000001
#define EINT1_EDGE   0x00000002
#define EINT2_EDGE   0x00000004
#define EINT3_EDGE   0x00000008

#define EINT0_RISING 0x00000001
#define EINT1_RISING 0x00000002
#define EINT2_RISING 0x00000004
#define EINT3_RISING 0x00000008

#define ENC_A_DENOM  50

extern void PrintAllEncoders(void);
extern void EncoderInit(void);
extern int32_t EncoderRead(uint32_t channel);

#endif /* ENCODER_H_ */
