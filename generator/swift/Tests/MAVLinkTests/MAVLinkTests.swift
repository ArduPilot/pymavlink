//
//  MAVLinkTests.swift
//  MAVLinkTests
//
//  Created by Max Odnovolyk on 10/6/16.
//  Copyright © 2016 Build Apps. All rights reserved.
//

import XCTest
@testable import MAVLink

class MAVLinkTests: XCTestCase {
    
    override func setUp() {
        super.setUp()
        // Put setup code here. This method is called before the invocation of each test method in the class.
    }
    
    override func tearDown() {
        // Put teardown code here. This method is called after the invocation of each test method in the class.
        super.tearDown()
    }
    
    var erroredIds = Set<UInt8>()
    var parsedIds = Set<UInt8>()
    var receivedCount = 0
    var failedToReceiveCount = 0
    var parsedCount = 0
    var failedToParse = 0
    
    func testExample() {
        let bundle = Bundle(for: type(of: self))
        let path = bundle.url(forResource: "flight", withExtension: "tlog")
        let data = try! Data(contentsOf: path!)
        let mavLink = MAVLink()
        mavLink.delegate = self
        
        for (offset, data) in data.enumerated() {
            let _ = offset
            let _ = mavLink.parse(char: data, channel: 0)
        }
        
        //mavLink.parse(data: data, channel: 0)
        
        print(erroredIds)
        print(parsedIds)
        print("received: \(receivedCount)")
        print("failed to receive: \(failedToReceiveCount)")
        print("parsed: \(parsedCount)")
        print("failed to parse: \(failedToParse)")
    }
    
    func testPerformanceExample() {
        // This is an example of a performance test case.
        self.measure {
            // Put the code you want to measure the time of here.
        }
    }
}

extension MAVLinkTests: MAVLinkDelegate {
    func didReceive(packet: Packet, on channel: Channel, via link: MAVLink) {
        receivedCount += 1
    }
    
    func didFailToReceive(packet: Packet?, with error: Error, on channel: Channel, via link: MAVLink) {
        failedToReceiveCount += 1
        
        switch error {
        case let ParseError.invalidPayloadLength(messageId, _, _):
            erroredIds.insert(messageId)
        default:
            
            break
        }
    }
    
    func didParse(message: Message, from packet: Packet, on channel: Channel, via link: MAVLink) {
        parsedIds.insert(packet.messageId)
        parsedCount += 1
    }
    
    func didFailToParseMessage(from packet: Packet, with error: Error, on channel: Channel, via link: MAVLink) {
        failedToParse += 1
    }
    
    func didFinalize(message: Message, from data: Data, on channel: Channel, in link: MAVLink) {
        
    }
}
