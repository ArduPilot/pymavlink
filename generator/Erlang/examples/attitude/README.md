attitude
=====

Examples use Mavlink library over serial port (ttyACM0) 115200.

Run
-----
    $ cd mavlink
    $ python3 -m pymavlink.tools.mavgen --lang=Erlang --wire-protocol=1.0 --output=$MAVERLPATH message_definitions/v1.0/ardupilotmega.xml
    $ cd pymavlink/generator/Erlang/examples/attitude
    $ cp $MAVERLPATH/mavlink.erl src
    $ cp $MAVERLPATH/mavlink.hrl src
    $ rebar3 shell
