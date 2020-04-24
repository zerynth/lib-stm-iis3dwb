//#define ZERYNTH_PRINTF
#include "zerynth.h"

//#define printf(...) vbl_printf_stdout(__VA_ARGS__)

int32_t iis3dwb_read_registers(int32_t spi, int32_t reg, uint8_t nbit,  uint8_t *read){
    int32_t err = ERR_OK;

    uint8_t i;
    uint8_t tosend[nbit+1];
    tosend[0] = (reg & 0x7F) | 0x80;
    for(i = 1; i < nbit + 1; i++) {
        tosend[i] = 0;
    }
    if (vhalSpiExchange(spi, tosend, read, nbit+1) != 0)
        err = ERR_IOERROR_EXC;

    return err;
}

C_NATIVE(_iis3dwb_write_reg8){
    NATIVE_UNWARN();
    int32_t spi;
    int32_t reg;
    int32_t value;
    int32_t err = ERR_OK;

    if (parse_py_args("iii", nargs, args, &spi, &reg, &value) != 3)
        return ERR_TYPE_EXC;

    *res = MAKE_NONE();

    RELEASE_GIL();
    
    uint8_t tosend[2] = { reg & 0x7F, value & 0xFF };
    if (vhalSpiExchange(spi, tosend, NULL, 2) != 0)
        err = ERR_IOERROR_EXC;

    ACQUIRE_GIL();
    return err;
}

C_NATIVE(_iis3dwb_read_reg8){
    NATIVE_UNWARN();
    int32_t spi;
    int32_t reg;
    int32_t err = ERR_OK;
    uint8_t toread[2];

    if (parse_py_args("ii", nargs, args, &spi, &reg) != 2)
        return ERR_TYPE_EXC;

    RELEASE_GIL();
    err = iis3dwb_read_registers(spi, reg, 1, toread);
    ACQUIRE_GIL();

    if (err == ERR_OK)
        *res = PSMALLINT_NEW(toread[1]);
    else
        *res = MAKE_NONE();
    return err;
}

C_NATIVE(_iis3dwb_write_reg16){
    NATIVE_UNWARN();
    int32_t spi;
    int32_t reg;
    int32_t value;
    int32_t err = ERR_OK;

    if (parse_py_args("iii", nargs, args, &spi, &reg, &value) != 3)
        return ERR_TYPE_EXC;

    *res = MAKE_NONE();

    RELEASE_GIL();
    
    uint8_t tosend[3] = { reg & 0x7F, value & 0xFF, (value >> 8) & 0xFF };
    if (vhalSpiExchange(spi, tosend, NULL, 3) != 0)
        err = ERR_IOERROR_EXC;

    ACQUIRE_GIL();
    return err;
}

C_NATIVE(_iis3dwb_read_reg16){
    NATIVE_UNWARN();
    int32_t spi;
    int32_t reg;
    uint8_t toread[3];
    int32_t err = ERR_OK;

    if (parse_py_args("ii", nargs, args, &spi, &reg) != 2)
        return ERR_TYPE_EXC;

    RELEASE_GIL();
    err = iis3dwb_read_registers(spi, reg, 2, toread);
    ACQUIRE_GIL();

    if (err == ERR_OK)
        *res = PSMALLINT_NEW((int16_t)(toread[1] + toread[2] * 256));
    else
        *res = MAKE_NONE();
    return err;
}


C_NATIVE(_iis3dwb_read_reg16x3){
    NATIVE_UNWARN();
    int32_t spi;
    int32_t reg;
    int32_t err = ERR_OK;
    uint8_t toread[7];

    if (parse_py_args("ii", nargs, args, &spi, &reg) != 2)
        return ERR_TYPE_EXC;

    RELEASE_GIL();
    err = iis3dwb_read_registers(spi, reg, 6, toread);
    ACQUIRE_GIL();

    if (err == ERR_OK) {
        PTuple* tpl = ptuple_new(3, NULL);
        PTUPLE_SET_ITEM(tpl,0,PSMALLINT_NEW((int16_t)(toread[1] + toread[2] * 256)));
        PTUPLE_SET_ITEM(tpl,1,PSMALLINT_NEW((int16_t)(toread[3] + toread[4] * 256)));
        PTUPLE_SET_ITEM(tpl,2,PSMALLINT_NEW((int16_t)(toread[5] + toread[6] * 256)));
        *res = tpl;
    }
    else
        *res = MAKE_NONE();
    return err;
}

C_NATIVE(_iis3dwb_getfast){
    NATIVE_UNWARN();

    int32_t spi;
    double sens;
    if (parse_py_args("if", nargs, args, &spi, &sens) != 2)
        return ERR_TYPE_EXC;

    RELEASE_GIL();
    int16_t temp;
    int16_t acc[3];
    iis3dwb_acquire(spi,&temp,acc);
    PTuple *tpl = ptuple_new(4,NULL);
    PTUPLE_SET_ITEM(tpl,0,pfloat_new((temp / 256.0) + 25.0));
    PTUPLE_SET_ITEM(tpl,1,pfloat_new(acc[0] * sens * 0.00981));
    PTUPLE_SET_ITEM(tpl,2,pfloat_new(acc[1] * sens * 0.00981));
    PTUPLE_SET_ITEM(tpl,3,pfloat_new(acc[2] * sens * 0.00981));
    *res = tpl;

    ACQUIRE_GIL();
    return ERR_OK;
}

int iis3dwb_acquire(int32_t spi, int16_t* temp, int16_t* acc){
    
    int8_t toread[15];
    int32_t err = ERR_OK;
    vhalSpiLock(spi);
    vhalSpiSelect(spi);

    err = iis3dwb_read_registers(spi, 0x20, 14, toread);

    if(err != ERR_OK) {
        vhalSpiUnselect(spi);
        vhalSpiUnlock(spi);
        return -1;
    }

    vhalSpiUnselect(spi);
    vhalSpiUnlock(spi);

    *temp = (toread[1] + toread[2] * 256.0);

    acc[0]  = (toread[9] + toread[10] * 256.0);
    acc[1]  = (toread[11] + toread[12] * 256.0); 
    acc[2]  = (toread[13] + toread[14] * 256.0); 

    return 0;
}
