autopilot_version
=====

Examples use Mavlink library over UDP (mavproxy 127.0.0.1:12345).

Run
-----
    $ cd mavlink
    $ python3 -m pymavlink.tools.mavgen --lang=Erlang --wire-protocol=1.0 --output=$MAVERLPATH message_definitions/v1.0/ardupilotmega.xml
    $ mavproxy.py --master=/dev/ttyACM0,115200 --out=udp:127.0.0.1:12345
    $ cd pymavlink/generator/Erlang/examples/autopilot_version
    $ cp $MAVERLPATH/mavlink.erl src
    $ cp $MAVERLPATH/mavlink.hrl src
    $ rebar3 shell
