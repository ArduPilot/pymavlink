-module(autopilot_version).
-behaviour(gen_server).

-include("autopilot_version.hrl").
-include("mavlink.hrl").

-export([start_link/0]).
-export([init/1, handle_call/3, handle_cast/2, handle_info/2,
         terminate/2, code_change/3]).

-define(SERVER, ?MODULE).

start_link() ->
  gen_server:start_link({local, ?SERVER}, ?MODULE, [], []).

init([]) ->
  {ok, Socket} = gen_udp:open(12345, [binary, {active, true}]),
  schedule_next_request(),
  {ok, #state{mav_conn=#mavlink_connection{system_id=255},
              udp_sock = Socket}}.

handle_call(_Request, _From, State) ->
  {reply, {error, unexpected}, State}.

handle_cast(_Request, State) ->
  {noreply, State}.

handle_info(tick, State) ->
  New_State = send_command_long_request_ap_version(State),
  schedule_next_request(),
  {noreply, New_State};

handle_info({udp, Socket, Address, Port, Data}, State) when Socket == State#state.udp_sock ->
  UDP_Buf = State#state.udp_buf,
  {Msgs, Buf} = mavlink:decode(<<UDP_Buf/binary, Data/binary>>),
  handle_mav(Msgs),

  {noreply, State#state{udp_addr=Address, udp_port=Port}};

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

handle_mav_msg(1, Autopilot_Version = #autopilot_version{}) ->
  io:format("Received autopilot_version: ~p\n", [Autopilot_Version]);

handle_mav_msg(_, _) ->
  unhandled.

schedule_next_request() ->
  erlang:send_after(1000, self(), tick).

send_command_long_request_ap_version(#state{udp_port = 0} = State) ->
  io:format("Error! Unknown mavproxy port~n"),
  State;

send_command_long_request_ap_version(State) ->
  Command_Long = #command_long{
    target_system = 1,
    target_component = 1,
    confirmation = 0,
    command = request_message,
    param1 = 148,
    param2 = 0,
    param3 = 0,
    param4 = 0,
    param5 = 0,
    param6 = 0,
    param7 = 0},

  Conn = State#state.mav_conn,
  {New_Conn, Buf} = mavlink:encode(Conn, Command_Long),

  gen_udp:send(State#state.udp_sock, State#state.udp_addr, State#state.udp_port, Buf),

  io:format("Send request: ~p~n", [Command_Long]),

  State#state{mav_conn=New_Conn}.
