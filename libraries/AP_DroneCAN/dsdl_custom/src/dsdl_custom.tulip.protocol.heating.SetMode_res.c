

#define CANARD_DSDLC_INTERNAL
#include <dsdl_custom.tulip.protocol.heating.SetMode_res.h>

#include <string.h>

#ifdef CANARD_DSDLC_TEST_BUILD
#include <test_helpers.h>
#endif

uint32_t dsdl_custom_tulip_protocol_heating_SetModeResponse_encode(struct dsdl_custom_tulip_protocol_heating_SetModeResponse* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
) {
    uint32_t bit_ofs = 0;
    memset(buffer, 0, DSDL_CUSTOM_TULIP_PROTOCOL_HEATING_SETMODE_RESPONSE_MAX_SIZE);
    _dsdl_custom_tulip_protocol_heating_SetModeResponse_encode(buffer, &bit_ofs, msg, 
#if CANARD_ENABLE_TAO_OPTION
    tao
#else
    true
#endif
    );
    return ((bit_ofs+7)/8);
}

/*
  return true if the decode is invalid
 */
bool dsdl_custom_tulip_protocol_heating_SetModeResponse_decode(const CanardRxTransfer* transfer, struct dsdl_custom_tulip_protocol_heating_SetModeResponse* msg) {
#if CANARD_ENABLE_TAO_OPTION
    if (transfer->tao && (transfer->payload_len > DSDL_CUSTOM_TULIP_PROTOCOL_HEATING_SETMODE_RESPONSE_MAX_SIZE)) {
        return true; /* invalid payload length */
    }
#endif
    uint32_t bit_ofs = 0;
    if (_dsdl_custom_tulip_protocol_heating_SetModeResponse_decode(transfer, &bit_ofs, msg,
#if CANARD_ENABLE_TAO_OPTION
    transfer->tao
#else
    true
#endif
    )) {
        return true; /* invalid payload */
    }

    const uint32_t byte_len = (bit_ofs+7U)/8U;
#if CANARD_ENABLE_TAO_OPTION
    // if this could be CANFD then the dlc could indicating more bytes than
    // we actually have
    if (!transfer->tao) {
        return byte_len > transfer->payload_len;
    }
#endif
    return byte_len != transfer->payload_len;
}

#ifdef CANARD_DSDLC_TEST_BUILD
struct dsdl_custom_tulip_protocol_heating_SetModeResponse sample_dsdl_custom_tulip_protocol_heating_SetModeResponse_msg(void) {

    struct dsdl_custom_tulip_protocol_heating_SetModeResponse msg;






    msg.success = (bool)random_bitlen_unsigned_val(1);





    return msg;

}
#endif
