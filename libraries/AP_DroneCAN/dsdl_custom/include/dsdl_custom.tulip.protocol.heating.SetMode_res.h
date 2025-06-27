
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <canard.h>




#define DSDL_CUSTOM_TULIP_PROTOCOL_HEATING_SETMODE_RESPONSE_MAX_SIZE 1
#define DSDL_CUSTOM_TULIP_PROTOCOL_HEATING_SETMODE_RESPONSE_SIGNATURE (0x1B6CECDB7E735039ULL)

#define DSDL_CUSTOM_TULIP_PROTOCOL_HEATING_SETMODE_RESPONSE_ID 202





#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
class dsdl_custom_tulip_protocol_heating_SetMode_cxx_iface;
#endif


struct dsdl_custom_tulip_protocol_heating_SetModeResponse {

#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
    using cxx_iface = dsdl_custom_tulip_protocol_heating_SetMode_cxx_iface;
#endif




    bool success;



};

#ifdef __cplusplus
extern "C"
{
#endif

uint32_t dsdl_custom_tulip_protocol_heating_SetModeResponse_encode(struct dsdl_custom_tulip_protocol_heating_SetModeResponse* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool dsdl_custom_tulip_protocol_heating_SetModeResponse_decode(const CanardRxTransfer* transfer, struct dsdl_custom_tulip_protocol_heating_SetModeResponse* msg);

#if defined(CANARD_DSDLC_INTERNAL)

static inline void _dsdl_custom_tulip_protocol_heating_SetModeResponse_encode(uint8_t* buffer, uint32_t* bit_ofs, struct dsdl_custom_tulip_protocol_heating_SetModeResponse* msg, bool tao);
static inline bool _dsdl_custom_tulip_protocol_heating_SetModeResponse_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct dsdl_custom_tulip_protocol_heating_SetModeResponse* msg, bool tao);
void _dsdl_custom_tulip_protocol_heating_SetModeResponse_encode(uint8_t* buffer, uint32_t* bit_ofs, struct dsdl_custom_tulip_protocol_heating_SetModeResponse* msg, bool tao) {

    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;






    canardEncodeScalar(buffer, *bit_ofs, 1, &msg->success);

    *bit_ofs += 1;





}

/*
 decode dsdl_custom_tulip_protocol_heating_SetModeResponse, return true on failure, false on success
*/
bool _dsdl_custom_tulip_protocol_heating_SetModeResponse_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct dsdl_custom_tulip_protocol_heating_SetModeResponse* msg, bool tao) {

    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;





    canardDecodeScalar(transfer, *bit_ofs, 1, false, &msg->success);

    *bit_ofs += 1;





    return false; /* success */

}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct dsdl_custom_tulip_protocol_heating_SetModeResponse sample_dsdl_custom_tulip_protocol_heating_SetModeResponse_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"

#ifdef DRONECAN_CXX_WRAPPERS
#include <canard/cxx_wrappers.h>



#endif
#endif
