#include "wap.h"

int wap_parse_header(uint32_t header, wap_header *out) {
    if ((header >> 16) != WAP_MAGIC) return 0;
    out->command = (uint8_t)(header & 0xFF);
    out->word_count = (uint8_t)((header >> 8) & 0xFF);
    out->is_response = (out->command & 0x80) ? 1 : 0;
    return 1;
}

const char *wap_command_name(uint8_t command) {
    switch (command & 0x7F) {
        case 0x10: return "Init1";
        case 0x11: return "Unknown";
        case 0x13: return "GetSomeValue";
        case 0x16: return "Broadcast";
        case 0x17: return "Setup";
        case 0x19: return "StartHost";
        case 0x1A: return "AcceptConnections";
        case 0x1B: return "EndHost";
        case 0x1C: return "BroadcastReadStart";
        case 0x1D: return "BroadcastReadPoll";
        case 0x1E: return "BroadcastReadEnd";
        case 0x1F: return "Connect";
        case 0x20: return "IsConnecting";
        case 0x21: return "FinishConnecting";
        case 0x24: return "SendData";
        case 0x25: return "SendDataAndWait";
        case 0x26: return "ReceiveData";
        case 0x27: return "ReceiveDataAndWait";
        case 0x28: return "ReceiveDataAndWaitResponse";
        case 0x3D: return "Init2";
        default: return "?";
    }
}
