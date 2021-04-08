%=============================================================================%
% YuMiScheduler
%
% Johan Ludde Wessén
% Latest Update 2021-02-21
%============================================================================%

include "globals.mzn";
include "gecode.mzn";

% model configuration flags
bool: implied_value_precede = true;
bool: implied_regular = false;
bool: implied_cumulative = false;
bool: implied_diffn = false;

int: no_agents = 2; % number of arms
int: no_locations = card(index_set_1of2(left_arm_travel_times) ); % number of locations
int: no_actual_tasks = card(index_set_2of2(task_durations)); % number of actual tasks, i.e. excluding dummy start & end tasks

int: no_tot_tasks = no_actual_tasks+2*no_agents; % all tasks, including dummy start & end tasks

int: min_duration = 1;
int: max_duration = max([ task_durations[r,n] | r in AGENTS, n in ACTUAL_TASKS]);
int: max_travel_time = max([ left_arm_travel_times[i,j] | i,j in LOCATIONS] ++ [ right_arm_travel_times[i,j] | i,j in LOCATIONS] );


int: time_budget_left = sum (i in ACTUAL_TASKS) (
  if -1 < max([left_arm_travel_times[l,l] | l in location_domain(i)]) then
    task_durations[1,i] + max([ left_arm_travel_times[l1,l2] | l1 in location_domain(i), l2 in LOCATIONS ])
  else
    0 % if we can't do the task, we shouldn't count it
  endif
);

int: time_budget_right = sum (i in ACTUAL_TASKS) (
  if -1 < max([right_arm_travel_times[l,l] | l in location_domain(i)]) then
    task_durations[2,i] + max([ right_arm_travel_times[l1,l2] | l1 in location_domain(i), l2 in LOCATIONS ])
  else
    0  % if we can't do the task, we shouldn't count it
  endif
);
int: time_budget = max([time_budget_left,time_budget_right]);


% Calculate the absolute minimum time an arm needs to work + travel
int: min_time_budget_left = sum (i in ACTUAL_TASKS) (
  if -1 < min([left_arm_travel_times[l,l] | l in location_domain(i)]) then
    task_durations[1,i] + min([ left_arm_travel_times[l1,l2] | l1 in location_domain(i), l2 in LOCATIONS ])
  else
    0 % if we can't do the task, we shouldn't count it
  endif
);

int: min_time_budget_right = sum (i in ACTUAL_TASKS) (
  if -1 < min([right_arm_travel_times[l,l] | l in location_domain(i)]) then
    task_durations[2,i] + min([ right_arm_travel_times[l1,l2] | l1 in location_domain(i), l2 in LOCATIONS ])
  else
    0  % if we can't do the task, we shouldn't count it
  endif
);
int: min_time_budget = min([min_time_budget_left,min_time_budget_right]);

% Calculate the time if one arm recieves all tasks it can perform, and manages to get minimal travel time
int: minimax_time_budget_left = sum (i in ACTUAL_TASKS) (
  if -1 < max([left_arm_travel_times[l,l] | l in location_domain(i)]) then
    task_durations[1,i] + min([ left_arm_travel_times[l1,l2] | l1 in location_domain(i), l2 in LOCATIONS ])
  else
    0 % if we can't do the task, we shouldn't count it
  endif
);

int: minimax_time_budget_right = sum (i in ACTUAL_TASKS) (
  if -1 < max([right_arm_travel_times[l,l] | l in location_domain(i)]) then
    task_durations[2,i] + min([ right_arm_travel_times[l1,l2] | l1 in location_domain(i), l2 in LOCATIONS ])
  else
    0  % if we can't do the task, we shouldn't count it
  endif
);

int: minimax_time_budget = min([minimax_time_budget_left,minimax_time_budget_right]);
int: min_traveltime = 1;
int: min_waittime = 1;
set of int: TOTTIME = 0..time_budget;
set of int: WAITTIME = min_waittime..time_budget;


set of int: DURTIME = min_duration..max_duration; %smaller domain, for task durations. needs to include 0, due to dummy tasks
set of int: TRAVELTIME = min_traveltime..max_travel_time; % smaller domain, for travel times, needs to include 0, due to dummy tasks

% The last tasks represent the start and end task for each vehicle (dummies, not depot)
set of int: AGENTS = 1..no_agents;
set of int: TASKS = 1..no_tot_tasks;
set of int: ACTUAL_TASKS = 1..no_actual_tasks;

set of int: LOCATIONS = 1..no_locations;


% Depot tasks are not actual tasks, but rather a modelling convenience, that separates our
% successors into 2 sequences
set of int: DEPOT_TASKS = no_actual_tasks+1..no_tot_tasks;
set of int: START_DEPOT_TASKS = no_actual_tasks+1..no_actual_tasks+no_agents;
set of int: END_DEPOT_TASKS = no_actual_tasks+no_agents+1..no_tot_tasks;

array[int] of int: DEPOT_TASK_LIST = [no_actual_tasks + a | a in 1..2*no_agents];
array[int] of int: START_DEPOT_TASK_LIST = [no_actual_tasks + a | a in AGENTS];
array[int] of int: END_DEPOT_TASK_LIST = [no_actual_tasks + no_agents + a | a in AGENTS];

%To be defined in datafile:
% TODO: generalize
set of int: TRAY_TASKS;
set of int: CAMERA_TASKS;
set of int: FIXTURE_TASKS = all_tasks(fixture_task_orders);
set of int: GRIPPER_TASKS = all_tasks(gripper_pick_tasks_orders);
set of int: SUCTION_TASKS = all_tasks(suction_pick_tasks_orders);
set of int: AIRGUN_TASKS;
set of int: OUTPUT_TASKS;
set of int: empty_gripper_tasks; % these are tasks for which the application requires empty gripper (e.g. "peeling of tape")


% We assume these sets have no overlap, and form the full set of locations
set of int: TRAY_LOCATIONS;
set of int: CAMERA_LOCATIONS;
set of int: FIXTURE_LOCATIONS;
set of int: AIRGUN_LOCATIONS;
set of int: OUTPUT_LOCATIONS;

% Data
array[AGENTS, int] of int: task_durations;
array[int,int] of int: left_arm_travel_times;
array[int,int] of int: right_arm_travel_times;
array[int] of int: location_order;

% =================================================%
% Variables
% =================================================%
% These variables each belongs to a task.
% =================================================%
%
% Since we model multiple (2) agents using 1 hamiltonian task circuit
% we add dummy tasks as "breaks" between each agent's task sequence.
% The allover sequence starts with the start task of agent 1, the tasks of agent 1,
% and finishes with the end task of agent1. It then continues with the start task
% of agent 2, the tasks of agent 2, and finishes with the end task of agent2.
% The end task of agent 2 then finalizes the circuit by "pointing to" the
% start task of agent 1.
%

array[TASKS] of var AGENTS: agent; % Which agent performs the task

% Route encodings - each of these encode the same information, but in different ways
array[TASKS] of var TASKS: successor; % Task sequence encoding where successor[i] = j means task j follows task i
array[TASKS] of var TASKS: task; % Task sequence encoding where task[i] = j means task j is i:th to be performed

% Time variables:
array[TASKS] of var TOTTIME: arrival_time; % when agent arrives at task
array[TASKS] of var TOTTIME: start_time; % when agent start working on task
array[TASKS] of var TOTTIME: end_time; % when agent finish working on task
array[TASKS] of var TOTTIME: next_arrival_time; % when agent arrives at next task

array[TASKS] of var WAITTIME: waiting_time; % Time between arrival_time and start_time (needed(?) to ensure collision detection at all times)
array[TASKS] of var DURTIME: duration; % duration of task
array[TASKS] of var TRAVELTIME: travel_time; % travel time of going from task i to successor[i]


% Locations - locations are variable
array[TASKS] of var LOCATIONS: location;
array[TASKS] of var LOCATIONS: next_location; % Used to induce traveltime between tasks



% =================================================%
% Variables not attributet to one task
% =================================================%

% META VARIABLES - used for search & increasing propagation
array[1..1,AGENTS] of var TASKS: agent_counts;
var TASKS: agent_count = 2*length(agent) - sum(agent); % # tasks assigned to left arm

constraint agent_counts[1,1] = 2*card(TRAY_TASKS) - sum(t in TRAY_TASKS)(agent[t]);
constraint agent_counts[1,2] = sum(t in TRAY_TASKS)(agent[t]) - card(TRAY_TASKS);

% =================================================%

% Routing Model Constraints:
%
% Each arm task sequence starts with a (dummy) start task, and a (dummy) end task
% Since it is all modelled as one Hamiltonian Circuit, we need to connect arm sequences to each other
% Thus,
% - After end task of arm n comes start task of arm n+1
% - After end task of arm N comes start task of arm 0
% We use 4 equivalent Encodings:
% (A) successor
% (B) task

% =================================================%
% Constraint related to the dummy tasks:
% [..., START_DEPOT_TASK_LIST[1], START_DEPOT_TASK_LIST[2], END_DEPOT_TASK_LIST[1], END_DEPOT_TASK_LIST[2]]
% =================================================%

% -----------------------------------------------------
% Constraints on dummy tasks' sequence encodings

% **********************
%Constraint 1, row 3:
% **********************
% Encoding A:
constraint successor[END_DEPOT_TASK_LIST[2]] = START_DEPOT_TASK_LIST[1] ;
constraint successor[END_DEPOT_TASK_LIST[1]] = START_DEPOT_TASK_LIST[2] ;

% Encoding B: (only states start & end of full sequence)START_DEPOT_TASK_LIST[1]] = END_DEPOT_TASK_LIST[2];

constraint task[no_tot_tasks] = END_DEPOT_TASK_LIST[2];
constraint task[1] = START_DEPOT_TASK_LIST[1]; % The statements A) There is "c" ones in "agent" and B) agent one's last task is on c:th place in the sequence
constraint task[agent_count] = END_DEPOT_TASK_LIST[1];
constraint task[agent_count+1] = START_DEPOT_TASK_LIST[2];

% Fixing dummy tasks to correct agent:
constraint forall(a in AGENTS)(
  agent[START_DEPOT_TASK_LIST[a]] = a
  /\
  agent[END_DEPOT_TASK_LIST[a]] = a
);

% Making sure we get the full cost of the schedule:
% Make an artificial "depot"
% Return the arm to 1:st position at the end of a schedule
% The last task in the sequence should reflect that it returns to the first task of the cycle
%  Thus, we make sure that the end task get the same location as the successor of the start task of the arm
%-------------------------------------------------------
% Constraint 1, row 5-6:
constraint forall(a in AGENTS) (
  location[END_DEPOT_TASK_LIST[a]] == location[START_DEPOT_TASK_LIST[a]]
  /\
  location[END_DEPOT_TASK_LIST[a]] = next_location[END_DEPOT_TASK_LIST[a]] % Missing from Constraint 1, row 5-6 (it should read location_succ_T-2 = location_T-2 and location_succ_T-1 = location_T-1)
  /\
  location[START_DEPOT_TASK_LIST[a]] = next_location[START_DEPOT_TASK_LIST[a]]
  /\
  next_location[START_DEPOT_TASK_LIST[a]] == location[successor[START_DEPOT_TASK_LIST[a]]]
);

%-------------------------------------------------------
% ----- Dummy Tasks Time Constraints ------------
%-------------------------------------------------------


% Overlapping cycles: ------------------------------------------
constraint forall(t in START_DEPOT_TASKS) (
  next_arrival_time[t] = arrival_time[successor[t]]
);

% At least one agent starts at time 0
constraint min(t in START_DEPOT_TASKS)(next_arrival_time[t]) = 0;
constraint max(t in START_DEPOT_TASKS)(next_arrival_time[t]) = cycle_overlap;

constraint forall(t in DEPOT_TASKS) (
    travel_time[t] == min_traveltime
    /\
    duration[t] == min_duration
    /\
    waiting_time[t] = min_waittime
  );


constraint forall(t in DEPOT_TASKS) (
    start_time[t] == arrival_time[t]
    /\
    start_time[t] == end_time[t]
    /\
    start_time[t] == next_arrival_time[t]
  );



%===========================================
% END DUMMY TASK CONSTRAINTS
%===========================================


%===========================================
% Route Model Constraints
%===========================================

% ---------------------------------------------------------------------
% Hamiltonian Circuit
%--------------------------------
constraint circuit(successor) :: domain;

% ----------------------------------------------------------------------
% Connect Successor Encoding (A) with "task" Encoding (B)
% ----------------------------------------------------------------------
constraint forall(t in 1..(no_tot_tasks-1)) (task[t+1] = successor[task[t]]);

% Redundant
constraint alldifferent(task) :: domain;


%-----------------------------------------------------------------------------------
% Connecting Successor Encoding (A) with Route Assignment:
% Posting that successors & predeccessors has same agent (agent)
%------------------------------------------------------------------------------------

constraint
   forall(t in ACTUAL_TASKS) (agent[t] == agent[successor[t]]);

constraint forall(t in START_DEPOT_TASKS) (agent[t] == agent[successor[t]]);

constraint forall(t in END_DEPOT_TASKS) (agent[t] != agent[successor[t]]);

% ==============================================
% Locations Constraints:
% ==============================================
int: no_fixture_locations = card(FIXTURE_LOCATIONS);

% Only allowing locations of correct type
predicate remove_invalids(set of int: tasks, set of int: locs) =
  forall(t in tasks) (location[t] in locs) :: domain_change_constraint;

% Pre Constraint 7
constraint remove_invalids(TRAY_TASKS, TRAY_LOCATIONS);
constraint remove_invalids(CAMERA_TASKS, CAMERA_LOCATIONS);
constraint remove_invalids(FIXTURE_TASKS, FIXTURE_LOCATIONS);
constraint remove_invalids(AIRGUN_TASKS, AIRGUN_LOCATIONS);
constraint remove_invalids(OUTPUT_TASKS, OUTPUT_LOCATIONS);


constraint forall(t in ACTUAL_TASKS) (next_location[t] = location[successor[t]]);

% All tasks of a fixture, must be done at the same fixture
constraint forall(f in 1..no_fixtures)(
  all_equal([location[fixture_task_orders[f,t]] | t in 1..fixture_order_lengths where fixture_task_orders[f,t] >= 0 ] )
);

% All tasks of different fixtures, must be done at different locations
constraint alldifferent([location[fixture_task_orders[f,1]] | f in 1..no_fixtures]);


% ----------------------------------------------------

% Some locations are only reachable by some agents
% Using that diagonal elements of travelling matrix is >=0 if feasible, -1 otherwise (currently only0, but might be >0 in the future)
% REDUNDANT wrt constraint below, since it puts exactly the same restrictions on all tasks' agent-location tuples (however, including this constraint seems to make things ever so slightly faster..)
constraint forall(t in TASKS)(
  let { array[int] of int: raw_extension =
    [ if x = 1 then 1 else l endif | l in location_domain(t) where left_arm_travel_times[l,l] >= 0, x in 1..2]
    ++
    [ if x = 1 then 2 else l endif | l in location_domain(t) where right_arm_travel_times[l,l] >= 0, x in 1..2]

    } in table( [agent[t]] ++ [location[t]], array2d(1..(length(raw_extension) div 2), 1..2, raw_extension) )
);

%----------------------------------------------------

%------------------------------------------------------
% Travel time depends on arm, location, next_location,
% Create table for a 4-tuple: [agent[t]] ++ [location[t]] ++ [next_location[t]] ++ [travel_time[t]]
constraint forall(t in ACTUAL_TASKS)(

  let { array[int] of int: raw_tt_extension = [
    if x = 1 then
      1
    elseif x = 2 then
      l1
    elseif x = 3 then
      l2
    else
      if l1 = l2 then
        min_traveltime
      else
        left_arm_travel_times[l1,l2]
      endif
    endif

    | l1 in location_domain(t) where left_arm_travel_times[l1,l1] >= 0, l2 in LOCATIONS where left_arm_travel_times[l2,l2] >= 0, x in 1..4]

    ++ [
    if x = 1 then
      2
    elseif x = 2 then
      l1
    elseif x = 3 then
      l2
    else
      if l1 = l2 then
        min_traveltime
      else
        right_arm_travel_times[l1,l2]
      endif
    endif

    | l1 in location_domain(t) where right_arm_travel_times[l1,l1] >= 0, l2 in LOCATIONS where right_arm_travel_times[l2,l2] >= 0, x in 1..4]

    } in table( [agent[t]] ++ [location[t]] ++ [next_location[t]] ++ [travel_time[t]], array2d(1..(length(raw_tt_extension) div 4), 1..4, raw_tt_extension)) :: defines_var(travel_time[t])
);

% ======================================
% ----- Core Time Constraints ------------ %
% ======================================

constraint forall(t in ACTUAL_TASKS) (
  start_time[t] + duration[t] == end_time[t]
);

constraint forall(t in ACTUAL_TASKS) (
  end_time[t] + travel_time[t] == next_arrival_time[t]
);

constraint forall(t in ACTUAL_TASKS) (
  next_arrival_time[t] = arrival_time[successor[t]]
);

constraint forall(t in ACTUAL_TASKS)(
  arrival_time[t] + waiting_time[t] = start_time[t]
);

% Table: arm dependent task duration
constraint forall(t in ACTUAL_TASKS) (
  table( [agent[t]] ++ [duration[t]], array2d(1..2, 1..2,
    [ if x = 1 then r else task_durations[r,t] endif | r in 1..2 , x in 1..2]))  );

%-----------------------------------------------------

% Constrain first task of fixture to start immiediately - could possibly hinder good _cyclic_ schedules
constraint forall(r in 1..no_fixtures)(
  start_time[fixture_task_orders[r,1]] = arrival_time[fixture_task_orders[r,1]] + min_waittime
);

% Constrain tasks to start directly, or as soon as previous task finishes
constraint forall(r in 1..no_fixtures, n in 1..(fixture_order_lengths - 1) where fixture_task_orders[r,n+1] >= 0 )(
  start_time[fixture_task_orders[r,n+1]] = max(arrival_time[fixture_task_orders[r,n+1]] + min_waittime,
                                               end_time[fixture_task_orders[r,n]])
);

% Constrain non-fixture tasks to work when arriving at location
% this is valid since non-fixture tasks occupy the same space while waiting as it does while working
constraint forall(t in ACTUAL_TASKS where not(t in FIXTURE_TASKS) ) (
  arrival_time[t] + min_waittime = start_time[t]
  /\
  waiting_time[t] = min_waittime
);

% Ordering imposed by suction pick-n-place
% Since we know these tasks are performed by same arm, we can use separat all times of  a task
constraint forall(order_list_id in index_set_1of2(suction_pick_tasks_orders), list_pos in 1..(suction_order_lengths - 1) where suction_pick_tasks_orders[order_list_id,list_pos+1] >= 0 )(
  next_arrival_time[suction_pick_tasks_orders[order_list_id,list_pos]] <= arrival_time[suction_pick_tasks_orders[order_list_id,list_pos+1]]
);

% Ordering imposed by gripper pick-n-place
% Since we know these tasks are performed by same arm, we can use separat all times of  a task
constraint forall(order_list_id in index_set_1of2(gripper_pick_tasks_orders), list_pos in 1..(gripper_order_lengths - 1) where gripper_pick_tasks_orders[order_list_id,list_pos+1] >= 0 )(
  next_arrival_time[gripper_pick_tasks_orders[order_list_id,list_pos]] <= arrival_time[gripper_pick_tasks_orders[order_list_id,list_pos+1]]
);

% -------------------------------------------------------------
% DISJUNCTIVE over arms
% -------------------------------------------------------------

% This is equal to posting one unary per arm
% Stricter disjunction (including duration and travel time) , since an arm task uses arm for both processing and transportation

constraint implied_constraint(
   if not implied_diffn then
     true
   else
     diffn(
       [arrival_time[t] | t in TASKS],
       [ agent[t] | t in TASKS],
       [ duration[t] + travel_time[t] + waiting_time[t] | t in TASKS],
       [1 | t in TASKS]
    )
  endif
 );

% All tasks
constraint implied_constraint(
  if not implied_cumulative then
    true
  else
    cumulative( [arrival_time[t] | t in ACTUAL_TASKS]
	      , [duration[t] + travel_time[t] + waiting_time[t] | t in ACTUAL_TASKS]
	      , [1 | t in ACTUAL_TASKS]
	      , 2
	      )
  endif
);

% Tasks of each arm
constraint implied_constraint(
  if not implied_cumulative then
    true
  else
    forall(a in AGENTS)(
      cumulative( [arrival_time[t] | t in ACTUAL_TASKS]
		, [duration[t] + travel_time[t] + waiting_time[t] | t in ACTUAL_TASKS]
		, [agent[t] = a | t in ACTUAL_TASKS]
		, 1
		)
    )
  endif
);

% =======================================================
% Suction Tool Task Arm Orderings
% =======================================================
int: no_suction_cups;

array[int,int] of int: suction_pick_tasks_orders;
int: no_suction_picks = max(index_set_1of2(suction_pick_tasks_orders));
int: suction_order_lengths = max(index_set_2of2(suction_pick_tasks_orders));


% -------------------------------------------------------
%  Ordering constraints on routing model
% -------------------------------------------------------

% Each pick-n-place belongs to same arm
constraint forall(s in 1..no_suction_picks)(
        all_equal([agent[suction_pick_tasks_orders[s,n]] | n in 1..(sequence_length(suction_pick_tasks_orders, s)) ] )
      );


% We know the ordering value will be increasing
constraint forall(order_list_id in index_set_1of2(suction_pick_tasks_orders)) (
  increasing([ arrival_time[suction_pick_tasks_orders[order_list_id,list_pos]] | list_pos in index_set_2of2(suction_pick_tasks_orders) where suction_pick_tasks_orders[order_list_id,list_pos] >= 0 ] )
);

% RegEx stating that tasks are done in certain order, on same arm
% It _does not_ make sure that the number of components in suction tool does not exceed capacity - that is taken care of in later regex constraint
constraint %implied_constraint(
if not implied_regular then
  true
elseif card(index_set_1of2(suction_pick_tasks_orders)) < 1 then % If we do not have suction pick task we don't need to worry about filling suction cups
  true
else
  forall(order_list_id in 1..no_suction_picks)(
  let {
    % Tasks that do not affect the component load and cannot be done without holding the tool
    string: keep_no_component = "[" ++ set2regex_str(TASKS diff all_tasks_of_sequence(suction_pick_tasks_orders, order_list_id)) ++ "]";

    % Tasks that do not affect the component load - except intermediate tasks that are treated separately
    string: keep_component_load = keep_no_component ;%"[" ++ set2regex_str(TASKS diff first_task_of_sequence(suction_pick_tasks_orders, order_list_id) diff last_task_of_sequence(suction_pick_tasks_orders, order_list_id) diff DEPOT_TASKS) ++ "]";

    string: adding_component_load = "[" ++ set2regex_str( first_task_of_sequence(suction_pick_tasks_orders, order_list_id)  ) ++ "]";
    string: performing_component_intermediate = "[" ++ set2regex_str( intermediate_tasks_of_sequence(suction_pick_tasks_orders, order_list_id)  ) ++ "]";

    int: seq_len = sequence_length(suction_pick_tasks_orders, order_list_id) ;

    %Create a string of intermediate values - interleaved with the string in "keep_component_load"
    % This forces that the tasks in the pick-and-place list are performed in the prescribed order, possibly with tasks in between
    array[int] of string: performing_component_intermediate_plus_keep_component_load_star = ["[" ++ set2regex_str( { suction_pick_tasks_orders[order_list_id, seq_ind ] } ) ++ "]" ++ keep_component_load ++ "*" | seq_ind in 2..(seq_len-1)];

    string: removing_component_load = "[" ++ set2regex_str( last_task_of_sequence(suction_pick_tasks_orders, order_list_id) ) ++ "]";

    string: strSuctionHold =
          "(" ++
              keep_no_component ++ "*" ++
              "(" ++
                  adding_component_load ++
                  keep_component_load ++ "*"   ++
                  %performing_component_intermediate ++ %merely forces one intermediate tasks to be done, not all...
                  str_a2str(performing_component_intermediate_plus_keep_component_load_star,1) ++
                  %keep_component_load ++ "*"   ++
                  removing_component_load ++
              ")?" ++
          ")*";

    string: strSuctionHold2 =
          "[" ++ set2regex_str({ START_DEPOT_TASK_LIST[1] }) ++ "]" ++
          "(" ++
              keep_no_component ++ "*" ++
              "(" ++
                  adding_component_load ++
                  keep_component_load ++ "*"   ++
                  %performing_component_intermediate ++ %merely forces one intermediate tasks to be done, not all...
                  str_a2str(performing_component_intermediate_plus_keep_component_load_star,1) ++
                  %keep_component_load ++ "*"   ++
                  removing_component_load ++
              ")?" ++
          ")*" ++
          "[" ++ set2regex_str({ END_DEPOT_TASK_LIST[1] }) ++ "]" ++
          "[" ++ set2regex_str({ START_DEPOT_TASK_LIST[2] }) ++ "]" ++
          "(" ++
              keep_no_component ++ "*" ++
              "(" ++
                  adding_component_load ++
                  keep_component_load ++ "*"   ++
                  %performing_component_intermediate ++ %merely forces one intermediate tasks to be done, not all...
                  str_a2str(performing_component_intermediate_plus_keep_component_load_star,1) ++
                  %keep_component_load ++ "*"   ++
                  removing_component_load ++
              ")?" ++
          ")*" ++
          "[" ++ set2regex_str( { END_DEPOT_TASK_LIST[2] } ) ++ "]"

          ;


  } in regular(task, strSuctionHold)
)
endif
%)
;

% Regex - suction cup holding/not holding _any_ component
% Regex - stating suction cups becomes filled when picking up stuff, and emptied when placing them
constraint %implied_constraint(
if not implied_regular then
  true
else
let {

string: start_tasks = "[" ++ set2regex_str(START_DEPOT_TASKS) ++ "]";
string: end_tasks = "[" ++ set2regex_str(END_DEPOT_TASKS) ++ "]";

string: keep_tool_empty = "[" ++ set2regex_str(TASKS diff all_tasks(suction_pick_tasks_orders)) ++ "]";
string: keep_tool_load = "[" ++ set2regex_str(TASKS diff all_first_tasks(suction_pick_tasks_orders) diff all_last_tasks(suction_pick_tasks_orders) diff DEPOT_TASKS) ++ "]";

string: adding_tool_load = "[" ++ set2regex_str(all_first_tasks(suction_pick_tasks_orders)) ++ "]";
string: removing_tool_load = "[" ++ set2regex_str(all_last_tasks(suction_pick_tasks_orders)) ++ "]";

bool: uses_tool = adding_tool_load != "[]" /\ removing_tool_load != "[]"; %check that the have equal no of elements


string: strSuctionHold =
  "(" ++
    start_tasks ++
      "(" ++
          keep_tool_empty ++ "*" ++
          if uses_tool then
            "(" ++
                adding_tool_load ++
                keep_tool_load ++ "*"   ++

                if no_suction_cups > 1 then
                  "(" ++
                      adding_tool_load ++
                      keep_tool_load ++ "*" ++
                      removing_tool_load ++
                      keep_tool_load ++ "*"   ++
                  ")*"
                else
                  ""
                endif ++

                removing_tool_load ++
            ")*"
          else
            ""
          endif ++
      ")*" ++

      end_tasks ++
      "){" ++ show(no_agents) ++ "}"

      ;

} in regular(task, strSuctionHold)
endif
%)
;


% ========================================================
% Gripper Tool Tasks Arm Orderings
% ========================================================
array[int,int] of int: gripper_pick_tasks_orders;
int: no_gripper_picks = card(index_set_1of2(gripper_pick_tasks_orders));
int: gripper_order_lengths = max(index_set_2of2(gripper_pick_tasks_orders));

% -------------------------------------------------------
% -------------------------------------------------------

% All pick-n-place belongs to same arm
constraint forall(order_list_id in 1..no_gripper_picks)(
        all_equal([agent[gripper_pick_tasks_orders[order_list_id,list_pos]] | list_pos in 1..(sequence_length(gripper_pick_tasks_orders, order_list_id)) ] )
      );

% Redundant wrt the pick-n-place
constraint forall(order_list_id in 1..no_gripper_picks) (
  increasing([ arrival_time[gripper_pick_tasks_orders[order_list_id,list_pos]] | list_pos in 1..(sequence_length(gripper_pick_tasks_orders, order_list_id)) ] )
);


% -------------------------------------------------------
%  Regex on routing model
% -------------------------------------------------------

% All gripper pnp regex:s
constraint %implied_constraint(
if not implied_regular then
  true
elseif card(index_set_1of2(gripper_pick_tasks_orders)) < 1 then
  true
else
  forall(order_list_id in 1..no_gripper_picks)(
    let {
      string: keep_tool_empty = "[" ++ set2regex_str(TASKS diff all_tasks_of_sequence(gripper_pick_tasks_orders, order_list_id)) ++ "]";

      % Tasks that does not affect the tool load, except from intermediate tasks that _must_ happen before we perform place-task
      string: keep_tool_load = "[" ++ set2regex_str(TASKS diff all_tasks_of_sequence(gripper_pick_tasks_orders, order_list_id) diff empty_gripper_tasks diff DEPOT_TASKS) ++ "]";

      string: adding_tool_load = "[" ++ set2regex_str( first_task_of_sequence(gripper_pick_tasks_orders, order_list_id)  ) ++ "]";
      string: performing_tool_intermediate = "[" ++ set2regex_str( intermediate_tasks_of_sequence(gripper_pick_tasks_orders, order_list_id)  ) ++ "]";

      int: seq_len = sequence_length(gripper_pick_tasks_orders, order_list_id) ;

      %Create a string of intermediate values - interleaved with the string in "keep_tool_load"
      % This forces that the tasks in the pick-and-place list are performed in the prescribed order, possibly with tasks in between
      array[int] of string: performing_tool_intermediate_plus_keep_tool_load_star = ["[" ++ set2regex_str( { gripper_pick_tasks_orders[order_list_id, seq_ind ] } ) ++ "]" ++ keep_tool_load ++ "*" | seq_ind in 2..(seq_len-1)];

      string: removing_tool_load = "[" ++ set2regex_str( last_task_of_sequence(gripper_pick_tasks_orders, order_list_id) ) ++ "]";

      bool: uses_tool = adding_tool_load != "[]" /\ removing_tool_load != "[]"; %check that the have equal no of elements

      string: strToolPnP =
            "(" ++
                keep_tool_empty ++ "*" ++
                  "(" ++
                      adding_tool_load ++
                      keep_tool_load ++ "*"   ++
                      % Below gives all intermediate tasks, interleaved with other tasks that keeps the tool load constant
                      str_a2str(performing_tool_intermediate_plus_keep_tool_load_star,1) ++
                      %keep_tool_load ++ "*"   ++
                      removing_tool_load ++
                  ")?"
                ++
                keep_tool_empty ++ "*" ++
            ")";
    } in regular(task, strToolPnP)
  )

endif
%)
;

% Ensuring gripper capacity (1) is not exceeded
constraint %implied_constraint(
if not implied_regular then
  true
else
  let {

    string: start_tasks = "[" ++ set2regex_str(START_DEPOT_TASKS) ++ "]";
    string: end_tasks = "[" ++ set2regex_str(END_DEPOT_TASKS) ++ "]";

    string: inEmptyGripperState = "[" ++ set2regex_str(TASKS diff all_tasks(gripper_pick_tasks_orders) diff DEPOT_TASKS ) ++ "]";
    string: inHoldingWithGripperState = "[" ++ set2regex_str(TASKS diff all_first_tasks(gripper_pick_tasks_orders) diff all_last_tasks(gripper_pick_tasks_orders) diff DEPOT_TASKS diff empty_gripper_tasks) ++ "]";

    string: fromEmptyToHoldingState = "[" ++ set2regex_str(all_first_tasks(gripper_pick_tasks_orders)) ++ "]";
    string: fromHoldingToEmptyState = "[" ++ set2regex_str(all_last_tasks(gripper_pick_tasks_orders)) ++ "]";


    string: strGripHold =
          "(" ++
            start_tasks ++
            "(" ++
              inEmptyGripperState ++ "*" ++
              "(" ++
                  fromEmptyToHoldingState ++
                  if inHoldingWithGripperState != "[]" then inHoldingWithGripperState ++ "*" else "" endif  ++
                  fromHoldingToEmptyState ++
              ")?" ++
            ")*" ++
            end_tasks ++
          "){"++show(no_agents)++"}"
          ;


    string: strGripHold2 =
            "[" ++ set2regex_str({ START_DEPOT_TASK_LIST[1] }) ++ "]" ++
            "(" ++
              inEmptyGripperState ++ "*" ++
              "(" ++
                  fromEmptyToHoldingState ++
                  if inHoldingWithGripperState != "[]" then inHoldingWithGripperState ++ "*" else "" endif  ++
                  fromHoldingToEmptyState ++
              ")?" ++
            ")*" ++
            "[" ++ set2regex_str({ END_DEPOT_TASK_LIST[1] }) ++ "]" ++
            "[" ++ set2regex_str({ START_DEPOT_TASK_LIST[2] }) ++ "]" ++
            "(" ++
              inEmptyGripperState ++ "*" ++
              "(" ++
                  fromEmptyToHoldingState ++
                  if inHoldingWithGripperState != "[]" then inHoldingWithGripperState ++ "*" else "" endif  ++
                  fromHoldingToEmptyState ++
              ")?" ++
            ")*" ++
            "[" ++ set2regex_str({ END_DEPOT_TASK_LIST[2] }) ++ "]"

          ;

  } in regular(task, strGripHold)

endif
%)
;


% redundant wrt above, however as one unified constraint as well as including holding constraint. It seems to be much more efficient
% (ensuring only 1 component at a time)
constraint %implied_constraint(
if not implied_regular then
  true
elseif card(index_set_1of2(gripper_pick_tasks_orders)) < 1 then
  true
else
  let {

    string: keep_tool_empty = "[" ++ set2regex_str(TASKS diff all_tasks(gripper_pick_tasks_orders)) ++ "]" ;
    string: keep_tool_load = "[" ++ set2regex_str(TASKS diff all_tasks(gripper_pick_tasks_orders) diff empty_gripper_tasks diff DEPOT_TASKS) ++ "]" ;

    string: keep_tool_load_star_safe = if keep_tool_load != "[]" then keep_tool_load ++ "*" else "" endif ;

    array[int] of string: adding_tool_load = ["[" ++ set2regex_str( first_task_of_sequence(gripper_pick_tasks_orders, order_list_id)  ) ++ "]" | order_list_id in 1..no_gripper_picks];
    array[int] of string: removing_tool_load = ["[" ++ set2regex_str( last_task_of_sequence(gripper_pick_tasks_orders, order_list_id) ) ++ "]"| order_list_id in 1..no_gripper_picks];

    int: max_seq_len = max_sequence_length(gripper_pick_tasks_orders, 1) ;

    array[int] of string: performing_tool_intermediate = ["[" ++ if sequence_length(gripper_pick_tasks_orders, order_list_id) - 1 < seq_ind then "" else set2regex_str( { gripper_pick_tasks_orders[order_list_id, seq_ind ] } ) endif ++ "]" | order_list_id in 1..no_gripper_picks, seq_ind in 2..(max_seq_len-1)];
    array[int,int] of string: performing_tool_intermediateS_safe = if max_seq_len > 2 then array2d(1..(length(performing_tool_intermediate)  div (max_seq_len-2)), 1..(max_seq_len-2), performing_tool_intermediate) else [|""|] endif ;

    array[int] of string: strToolPnP = [

          "(" ++
              keep_tool_empty ++ "*" ++
              "(" ++
                  adding_tool_load[order_list_id] ++
                  %performing_tool_intermediate[order_list_id] ++ %does not handle multiple intermediate tasks .. hence see below
                  keep_tool_load_star_safe ++
                  str_a_slice2str(performing_tool_intermediateS_safe, keep_tool_load_star_safe, order_list_id, 1, sequence_length(gripper_pick_tasks_orders, order_list_id) - 2) ++
                  removing_tool_load[order_list_id] ++
              ")?" ++
          ")*"


          |  order_list_id in 1..no_gripper_picks];
    } in regular(task, "(" ++ str_a2str(strToolPnP,1) ++ ")*" )

endif
%)
;


% =====================================================================
% Fixture orderings - regex for agent model + interval constraints
% =====================================================================
array[int,int] of int: fixture_task_orders;
int: no_fixtures = max(index_set_1of2(fixture_task_orders));
int: fixture_order_lengths = max(index_set_2of2(fixture_task_orders));

% -------------------------------------------------------
% ------------------------------------------------------

% Constraint 4, row 1
% Time constraints are posted with other time constraints (above)

% -------------------------------------------------------
%  Regex on routing model
% -------------------------------------------------------

% Constraint 4, row 1
%Regex - fixture order
% Given that an agent performs tasks on a fixture, those tasks are performed in the prescibed order
constraint %implied_constraint(

  if not implied_regular then
    true
  else
  forall(order_list_id in 1..no_fixtures)(
  let {

    string: start_tasks = "[" ++ set2regex_str(START_DEPOT_TASKS) ++ "]";
    string: end_tasks = "[" ++ set2regex_str(END_DEPOT_TASKS) ++ "]";

    % Tasks that are allowed to be done in between fixture tasks. TODO: This could be made with other constraints in mind - e.g. it is not allowed to pick up an item after droping it on fixture
    string: allowed_interspersed = "[" ++ set2regex_str(TASKS diff all_tasks_of_sequence(fixture_task_orders, order_list_id) diff DEPOT_TASKS) ++ "]";

    int: seq_len = sequence_length(fixture_task_orders, order_list_id) ;

    %Create a string of intermediate values - interleaved with the string in "allowed_interspersed"
    % This forces that the tasks in the pick-and-place list are performed in the prescribed order, possibly with tasks in between
    array[int] of string: performing_intermediate_interspersed_star = ["[" ++ set2regex_str( { fixture_task_orders[order_list_id, seq_ind ] } ) ++ "]?" ++ allowed_interspersed ++ "*" | seq_ind in 1..seq_len];

    string: strFixtureTasks =
          "(" ++
              start_tasks ++
              allowed_interspersed ++ "*" ++
              str_a2str(performing_intermediate_interspersed_star,1) ++
              end_tasks ++
          "){"++show(no_agents)++"}"
          ;

  } in regular(task, strFixtureTasks)
)
  endif
%)
;

% -----------------------------------------------------------
% END FIXTURE ORDERS
% -----------------------------------------------------------


% =============================================================
% Constraint Specific to the reality of assembly layout
% =============================================================

% All component trays need to be at different locations
constraint alldifferent([location[t] | t in TRAY_TASKS] );

% =============================================================
% END Assembly Layout Constraint
% =============================================================

% =============================================================
% Collision Coordination
% =============================================================

% Constraint 8, and equivalents - in separate file(s)

array[1..2] of int: FixtureWorkObstruction;

% Making sure fixture is emptied before next assembly:
constraint forall(order_list_id in 1..no_fixtures) (
    end_time[fixture_task_orders[order_list_id, sequence_length(fixture_task_orders, order_list_id)]] <= start_time[fixture_task_orders[order_list_id,1]] + period
  );

% Making sure no overlap of assembly tasks within an agent (might remove some solutions, but a reasonable simplification as to facilitate pick-up-and-delivery constraints hold):
constraint forall(a in AGENTS)(
  start_time[START_DEPOT_TASK_LIST[a]] + period = start_time[END_DEPOT_TASK_LIST[a]]
);

% =============================================================
% END Collision Coordination
% =============================================================

% =============================================================
% START Symmetry Breaking
% =============================================================

% None so far..

% =============================================================
% END Symmetry Breaking
% =============================================================


% =====================================
% Objective - and related constraints
% =====================================
constraint makespan = max( next_arrival_time );

constraint next_arrival_time[END_DEPOT_TASK_LIST[1]] >= (min_time_budget div 2) ;
constraint next_arrival_time[END_DEPOT_TASK_LIST[2]] >= (min_time_budget div 2) ;

% cycle overlap is the time between the finish time of one arm and the finish time of the other arm
% (it is also the time between the start time of one arm and the start time of the other arm)

var TOTTIME: cycle_overlap ;
constraint cycle_overlap = max([start_time[START_DEPOT_TASK_LIST[1]], start_time[START_DEPOT_TASK_LIST[2]] ]);

set of int: FINISH_TIMES = (minimax_time_budget div 2)..time_budget;
var FINISH_TIMES: period ;
var FINISH_TIMES: makespan;

constraint makespan = period + cycle_overlap ;

%constraint makespan <= sum(duration ++ travel_time);


% This follows from the definition of makespan, cycle_overlap and period
% Very useful, since this is a cost function without negative weights
constraint period = min([  next_arrival_time[END_DEPOT_TASK_LIST[1]] ,
                                    next_arrival_time[END_DEPOT_TASK_LIST[2]]
                                 ]) ;

% ============================================================================
% ============================================================================

% Enforce sequencing of fixture tasks
constraint
   if not implied_value_precede then
    true
  else
  forall( r in 1..no_fixtures,
          i in 1..fixture_order_lengths,
          j in 1..fixture_order_lengths
	where
	  j > i /\ fixture_task_orders[r,j] >= 1
	)(
    let {TASKS: ti = fixture_task_orders[r,i],
         TASKS: tj = fixture_task_orders[r,j],
	 var int: enable = (agent[ti] != agent[tj]) * ti,
    } in value_precede(ti, tj, [enable] ++ task)
  )
  endif
;

% Enforce sequencing of suction PnP tasks
constraint
   if not implied_value_precede then
    true
  else
  forall(r in index_set_1of2(suction_pick_tasks_orders))(
    value_precede_chain(suction_pick_tasks_orders[r,..], task)
  )
  endif
;

% Enforce sequencing of gripper PnP tasks
constraint
   if not implied_value_precede then
    true
  else
  forall(r in index_set_1of2(gripper_pick_tasks_orders))(
    value_precede_chain(gripper_pick_tasks_orders[r,..], task)
  )
  endif
;

% Gripper capacity constraint /\ empty gripper constraint
constraint
  let {array[TASKS] of var 0..1: load,
       array[TASKS] of int: delta =
         [ if has_element(t, col(gripper_pick_tasks_orders,1)) then 1
           elseif has_element(t, col(gripper_pick_tasks_orders,2)) then -1
	   else 0 endif
	 | t in TASKS
	 ],
    } in load[1] = 0 /\
         forall(j in TASKS where j>1)(
           load[j] = load[j-1] + delta[task[j]] /\
	   if task[j] in empty_gripper_tasks then
	     load[j] = 0
	   else true endif
         );

% Suction capacity constraint
constraint
  let {array[TASKS] of var 0..no_suction_cups: load,
       array[TASKS] of int: delta =
         [ if has_element(t, col(suction_pick_tasks_orders,1)) then 1
           elseif has_element(t, col(suction_pick_tasks_orders,3)) then -1
	   else 0 endif
	 | t in TASKS
	 ],
    } in load[1] = 0 /\
         forall(j in TASKS where j>1)(
           load[j] = load[j-1] + delta[task[j]]
         );

% This constraint improves propagation
constraint
  (  agent[OUTPUT_TASKS[1]] == 1
  /\ successor[OUTPUT_TASKS[1]] == END_DEPOT_TASK_LIST[1]
  /\ successor[START_DEPOT_TASK_LIST[2]] in TRAY_TASKS
  ) \/
  (  agent[OUTPUT_TASKS[1]] == 2
  /\ successor[OUTPUT_TASKS[1]] == END_DEPOT_TASK_LIST[2]
  /\ successor[START_DEPOT_TASK_LIST[1]] in TRAY_TASKS
  ) ;

%----------------------------------------------------


% ===================================== %
% Output
% ===================================== %

output
   [ "\nno_actual_tasks = "] ++ [show([no_actual_tasks])] ++
   [ "\nno_locations = "] ++ [show([no_locations])] ++
   [ "\nno_fixtures = "] ++ [show([no_fixtures])] ++

   [ "\nperiod = " ] ++ [show(period)]  ++
   [ "\nmakespan = "] ++ [show(makespan)] ++
   [ "\nagent = " ] ++ [ show(agent) ] ++
   [ "\ntask = " ] ++ [ show(task) ]++
   [ "\nlocation = " ] ++ [ show(location) ]++
   [ "\ntask_ordering = " ] ++ [ show([0 | t in TASKS]) ]++
   %[ "\ntask_ordering = " ] ++ [ show(task_ordering) ]++

   [ "\narrival_time = " ] ++ [ show(arrival_time) ]  ++
   [ "\nnext_arrival_time = " ] ++ [ show(next_arrival_time) ]  ++
   [ "\nstart_time = " ] ++ [ show(start_time) ]  ++
   [ "\nend_time = " ] ++ [ show(end_time) ]  ++
   [ "\ntravel_time = " ] ++ [ show(travel_time) ]  ++

   [ "\n"]
;

%----------------------------------------------------------------
% Helper functions for orderings (name suggests functionality)
%----------------------------------------------------------------

% Get last tasks from given sequence, of a matrix of task_orders
function set of int: first_task_of_sequence(array[int,int] of int: tasks_orders, int: c) = {
  tasks_orders[c,1] } ;

% Get intermediate tasks from given sequence, of a matrix of task_orders
function set of int: intermediate_tasks_of_sequence(array[int,int] of int: tasks_orders, int: c) = {
  tasks_orders[c,j] | j in 2..(card(index_set_2of2(tasks_orders)) - 1) where tasks_orders[c,j+1] > 0 };

% Get last tasks from given sequence, of a matrix of task_orders
function set of int: last_task_of_sequence(array[int,int] of int: tasks_orders, int: c) = {
  tasks_orders[c,j] | j in 2..(card(index_set_2of2(tasks_orders)) - 1) where tasks_orders[c,j] > 0 /\ tasks_orders[c,j+1] < 0 };

% Get all first tasks from a matrix of task_orders
function set of int: all_first_tasks(array[int,int] of int: task_orders) = {
  task_orders[i,1] | i in index_set_1of2(task_orders) } ;

% Get all last tasks from a matrix of task_orders
function set of int: all_last_tasks(array[int,int] of int: task_orders) =
    if card(index_set_1of2(task_orders)) = 0 then {} else all_last_tasks(task_orders, card(index_set_1of2(task_orders))) endif;

% Implementation of above (+ parameter pointing to each sequence, used for recursion)
function set of int: all_last_tasks(array[int,int] of int: tasks_orders, int: c) =
    if c == 1 then last_task_of_sequence(tasks_orders, c) else all_last_tasks(tasks_orders, c-1) union last_task_of_sequence(tasks_orders, c) endif;


% Return number of values in the sequence until we meet a -1 , i.e. index of last value = # of useful numbers in sequences
function int: max_sequence_length(array[int,int] of int: task_orders, int: sequence_no) =
    if card(index_set_1of2(task_orders)) < sequence_no then 0 else max(sequence_length(task_orders, sequence_no),max_sequence_length(task_orders, sequence_no+1)) endif;


% Return number of values in the sequence until we meet a -1 , i.e. index of last value = # of useful numbers in sequences
function int: sequence_length(array[int,int] of int: task_orders, int: sequence_no) =
    if card(index_set_1of2(task_orders)) = 0 then 0 else sequence_length(task_orders, sequence_no, 1) endif;

% Implementation of above, but requires additional index as input
function int: sequence_length(array[int,int] of int: tasks_orders, int: sequence_no, int: index) =
    if tasks_orders[sequence_no, index] < 0 then index-1 else sequence_length(tasks_orders, sequence_no, index+1) endif;

% Return all tasks of sequence
function set of int: all_tasks_of_sequence(array[int,int] of int: task_orders, int: c) = {
    task_orders[c,j] | j in index_set_2of2(task_orders) where task_orders[c,j] > 0 };

% Return all tasks of table
function set of int: all_tasks(array[int,int] of int: task_orders) =
    if card(index_set_1of2(task_orders)) = 0 then {} else all_tasks(task_orders, card(index_set_1of2(task_orders))) endif;

% Implementation of above
function set of int: all_tasks(array[int,int] of int: task_orders, int: c) =
    if c <= 1 then all_tasks_of_sequence(task_orders, c) else all_tasks(task_orders, c-1) union all_tasks_of_sequence(task_orders, c) endif;

% We often need the location domain of tasks, this should provide that
function set of int: location_domain(int: task) =
  if task in TRAY_TASKS then
    TRAY_LOCATIONS
  elseif task in CAMERA_TASKS then
    CAMERA_LOCATIONS
  elseif task in AIRGUN_TASKS then
    AIRGUN_LOCATIONS
  elseif task in OUTPUT_TASKS then
    OUTPUT_LOCATIONS
  else
    LOCATIONS % The task does not belong to the categories.. should be start or end task
  endif;


function string: set2regex_str(set of int: the_set) =
    set2regex_str(the_set, 1);

function string: set2regex_str(set of int: the_set, int: c) =
  if c > no_actual_tasks + 4 then
    ""
  elseif c in the_set then
    show(c) ++ " " ++ set2regex_str(the_set, c + 1)
  else
    set2regex_str(the_set, c + 1)
  endif;


function string: str_a2str(array[int] of string: string_array, int: c) =
    if card(index_set(string_array)) < c then "" else string_array[c] ++ str_a2str(string_array, c+1) endif;

function string: str_a_slice2str(array[int,int] of string: string_array, string: intersperse_str, int: seq_no, int: ind_no, int: max_no) =
    if ind_no > max_no then "" else string_array[seq_no, ind_no] ++ intersperse_str ++ str_a_slice2str(string_array, intersperse_str, seq_no, ind_no+1, max_no) endif ;


solve :: seq_search([
		     int_search(agent, dom_w_deg, indomain_random, complete),
		     int_search(location, dom_w_deg, indomain_min, complete),
		     int_search(task, input_order, indomain_min, complete),
		     int_search(arrival_time, smallest, indomain_split, complete),
                    ])
    :: restart_luby(100)
  minimize period;
