-module(attitude).
-behaviour(gen_server).

-include("attitude.hrl").
-include("mavlink.hrl").

-export([start_link/0]).
-export([init/1, handle_call/3, handle_cast/2, handle_info/2,
         terminate/2, code_change/3]).

-define(SERVER, ?MODULE).

start_link() ->
  gen_server:start_link({local, ?SERVER}, ?MODULE, [], []).

init([]) ->
  {ok, Serial} = gen_serial:open("/dev/ttyACM0", [{packet, none}, {baud, 115200}, {flow_control, none}]),
  {ok, #state{mav_conn=#mavlink_connection{system_id=255},
              serial_port = Serial}}.

handle_call(_Request, _From, State) ->
  {reply, {error, unexpected}, State}.

handle_cast(_Request, State) ->
  {noreply, State}.

handle_info({serial, _Info, Data}, State) ->
  Serial_Buf = State#state.serial_buf,
  {Msgs, Buf} = mavlink:decode(<<Serial_Buf/binary, Data/binary>>),
  handle_mav(Msgs),
  {noreply, State#state{serial_buf = Buf}};

handle_info(_Info, State) ->
  {noreply, State}.

terminate(_Reason, _State) ->
  ok.

code_change(_OldVsn, State, _Extra) ->
  {ok, State}.

% % -----------------------------------------------------------------

handle_mav([]) ->
  ok;

handle_mav([{Msg_Info, Msg} | Other]) ->
  handle_mav_msg(Msg_Info#msg_info.sys_id, Msg),

  handle_mav(Other);

handle_mav([Error | Other]) ->
  io:format("Bad msg: ~p~n", [Error]),

  handle_mav(Other).

% % handle_mav_msg

handle_mav_msg(1, Attitude = #attitude{}) ->
  io:format("Pitch: ~.5f   Roll: ~.5f   Yaw: ~.5f\n", [Attitude#attitude.pitch, Attitude#attitude.roll, Attitude#attitude.yaw]);

handle_mav_msg(_, _) ->
  unhandled.
