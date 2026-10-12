%%%-------------------------------------------------------------------
%% @doc autopilot_version public API
%% @end
%%%-------------------------------------------------------------------

-module(autopilot_version_app).

-behaviour(application).

-export([start/2, stop/1]).

start(_StartType, _StartArgs) ->
    autopilot_version_sup:start_link().

stop(_State) ->
    ok.

%% internal functions
