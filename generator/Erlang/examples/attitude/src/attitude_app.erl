%%%-------------------------------------------------------------------
%% @doc attitude public API
%% @end
%%%-------------------------------------------------------------------

-module(attitude_app).

-behaviour(application).

-export([start/2, stop/1]).

start(_StartType, _StartArgs) ->
    attitude_sup:start_link().

stop(_State) ->
    ok.

%% internal functions
