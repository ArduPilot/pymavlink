#import <Foundation/Foundation.h>
#import "MVMessageCommandLong.h"
#include <assert.h>
int main(void) {
    @autoreleasepool {
        uint32_t sources[] = {42, 0xABCDEF12};
        uint32_t targets[] = {0, 7, 255, 256, 0xFFFFFFFF};
        for (unsigned i=0;i<2;i++) for (unsigned j=0;j<5;j++) {
            MVMessageCommandLong *message = [[MVMessageCommandLong alloc]
                initWithSystemId:sources[i] componentId:11 targetSystem:targets[j]
                targetComponent:250 command:MAV_CMD_MISSION_START confirmation:1
                param1:1 param2:2 param3:3 param4:4 param5:5 param6:6 param7:7];
#ifdef MAVLINK_IFLAG_SYSID32
            assert(message != nil && message.systemId == sources[i] && message.targetSystem == targets[j]);
            NSData *bytes = message.data;
            mavlink_message_t rx = {0}, decoded = {0};
            mavlink_status_t status = {0}, result = {0};
            unsigned accepted = 0;
            for (NSUInteger k=0;k<bytes.length;k++) {
                if (mavlink_frame_char_buffer(&rx, &status, ((const uint8_t *)bytes.bytes)[k], &decoded, &result) == MAVLINK_FRAMING_OK) accepted++;
            }
            assert(accepted == 1);
            MVMessageCommandLong *copy = [[MVMessageCommandLong alloc] initWithCMessage:decoded];
            assert(copy.systemId == sources[i] && copy.targetSystem == targets[j] && copy.param7 == 7);
#else
            if (sources[i] > 255 || targets[j] > 255) assert(message == nil);
            else assert(message.systemId == sources[i] && message.targetSystem == targets[j]);
#endif
        }
    }
    return 0;
}
