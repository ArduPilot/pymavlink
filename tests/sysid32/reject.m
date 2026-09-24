#import <Foundation/Foundation.h>
#import "MVMavlink.h"
#include <assert.h>

@interface Receiver : NSObject <MVMavlinkDelegate>
@property(nonatomic) unsigned count;
@end
@implementation Receiver
- (void)mavlink:(MVMavlink *)link didGetMessage:(id<MVMessage>)message {
    assert(message.message.msgid == 0 && message.message.sysid == 42);
    self.count++;
}
- (MAV_BOOL)mavlink:(MVMavlink *)link shouldWriteData:(NSData *)data { return YES; }
@end

int main(int argc, char **argv) {
    @autoreleasepool {
        NSString *root = [NSString stringWithUTF8String:argv[1]];
        for (NSString *file in [[NSFileManager defaultManager] contentsOfDirectoryAtPath:root error:nil]) {
            if (![file hasSuffix:@".v2"]) continue;
            MVMavlink *parser = [[MVMavlink alloc] init];
            Receiver *receiver = [[Receiver alloc] init];
            parser.delegate = receiver;
            NSData *data = [NSData dataWithContentsOfFile:[root stringByAppendingPathComponent:file]];
            for (NSUInteger i = 0; i < data.length; i++) {
                [parser parseData:[data subdataWithRange:NSMakeRange(i, 1)]];
            }
            assert(receiver.count == 1);
        }
        for (unsigned flags = 2; flags < 8; flags++) {
            mavlink_message_t msg = {0};
            msg.magic = MAVLINK_STX;
            msg.incompat_flags = flags;
            assert([[MVMessage alloc] initWithCMessage:msg] == nil);
        }
    }
    return 0;
}
