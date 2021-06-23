% MIT License
%
% Copyright (c) 2021 Johan Ludde Wessén
%
% Permission is hereby granted, free of charge, to any person obtaining 
% a copy of this software and associated documentation files (the "Software"),
% to deal in the Software without restriction, including without limitation 
% the rights to use, copy, modify, merge, publish, distribute, sublicense, 
% and/or sell copies of the Software, and to permit persons to whom the 
% Software is furnished to do so, subject to the following conditions:
% 
% The above copyright notice and this permission notice shall be included 
% in all copies or substantial portions of the Software.
%
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, 
% EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
% MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
% NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
% LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
% OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
% WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.


%=============================================================================%
% YuMiScheduler Collision Model
%
% Johan Ludde Wessén
% Latest Update 2021-02-21
%============================================================================%
bool: using_cumulative_per_zone = true;
bool: using_diffn_per_zone = false;
bool: using_unified_diffn = false;

int: max_lwait = max( [ max(e) | e in wait_zones_left_data]  );
int: max_rwait = max( [ max(e) | e in wait_zones_right_data] );
int: max_lwork = max( [ max(e) | e in work_zones_left_data]  );
int: max_rwork = max( [ max(e) | e in work_zones_right_data] );


int: max_zones = max([max_lwait, max_rwait, max_lwork, max_rwork]);

set of int: ZONES = 0..max_zones;
set of int: FIXTURE_ZONES = (max_zones - no_fixture_locations + 1)..max_zones; %by design the fixture zones are last
set of int: ZONES_DIFF_FIXTURES = ZONES diff FIXTURE_ZONES;


array[int] of set of int:  wait_zones_left_data;
array[int] of set of int:  wait_zones_right_data;
array[int] of set of int:  work_zones_left_data;
array[int] of set of int:  work_zones_right_data;

array[int,int] of set of int: travel_zones_left_data;
array[int,int] of set of int: travel_zones_right_data;

% We are assigning 1 if any location in the set of location and next location blocks the current location-zone
array [ACTUAL_TASKS, ZONES] of var 0..1: task_blocker;

% Table to assign if a zone is occupied or blocked (for the other arm) by a travel between 2 locations
% This table is generated from the data, simulating 2 armed YuMi assembly cell described in the paper [To be inserted]
constraint forall(t in ACTUAL_TASKS, z in ZONES) (

  let { array[int] of int: task_blocker_data_raw = [

    if x = 1 then
      1
    elseif x = 2 then
      l1
    else
      if z in work_zones_left_data[l1] then
          1
        else
          0
        endif
    endif
    | l1 in location_domain(t) where left_arm_travel_times[l1,l1] >= 0, x in 1..3]

    ++ [
      if x = 1 then
        2
      elseif x = 2 then
        l1
      else
        if z in work_zones_right_data[l1] then
            1
          else
            0
          endif
      endif
      | l1 in location_domain(t) where right_arm_travel_times[l1,l1] >= 0, x in 1..3]

    } in table( [agent[t]] ++ [location[t]] ++ [task_blocker[t,z]], array2d(1..(length(task_blocker_data_raw) div 3), 1..3, task_blocker_data_raw))
);

% Analogous to work_blocker, but for waiting
array [ACTUAL_TASKS, ZONES] of var 0..1: wait_blocker;

% Table to assign if a zone is occupied while waiting at a location
constraint forall(t in ACTUAL_TASKS, z in ZONES) (

  let { array[int] of int: wait_blocker_data_raw = [

    if x = 1 then
      1
    elseif x = 2 then
      l1
    else
      if z in wait_zones_left_data[l1] then
          1
        else
          0
        endif
    endif
    | l1 in location_domain(t) where left_arm_travel_times[l1,l1] >= 0, x in 1..3]

    ++ [
      if x = 1 then
        2
      elseif x = 2 then
        l1
      else
        if z in wait_zones_right_data[l1] then
            1
          else
            0
          endif
      endif
      | l1 in location_domain(t) where right_arm_travel_times[l1,l1] >= 0, x in 1..3]

    } in table( [agent[t]] ++ [location[t]] ++ [wait_blocker[t,z]], array2d(1..(length(wait_blocker_data_raw) div 3), 1..3, wait_blocker_data_raw))
);



% Analougous to work_blocker
array [ACTUAL_TASKS, ZONES] of var 0..1: travel_blocker;

% Table to assign if a zone is occupied or blocked by a travel between 2 locations
constraint forall(t in ACTUAL_TASKS, z in ZONES) (

  let { array[int] of int: travel_blocker_data_raw = [

    if x = 1 then
      1
    elseif x = 2 then
      l1
    elseif x = 3 then
      l2
    else
      if z in travel_zones_left_data[l1,l2] then
        1
      else
        0
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
      if z in travel_zones_right_data[l1,l2] then
        1
      else
        0
      endif
    endif
    | l1 in location_domain(t) where right_arm_travel_times[l1,l1] >= 0, l2 in LOCATIONS where right_arm_travel_times[l2,l2] >= 0, x in 1..4]

    } in table( [agent[t]] ++ [location[t]] ++ [next_location[t]] ++ [travel_blocker[t,z]], array2d(1..(length(travel_blocker_data_raw) div 4), 1..4, travel_blocker_data_raw))
);


%------------------
% This is the default (full) formulation, however this does _not_ work well if
% any of the variables in the second argument (dur) has zero in its domain
% Having zero in such domain gives a very poor decomposition when using the default solver (Gecode)

% Note: If any variable of the second argument needs to be zero, there are other, less
% intuitive formulations that are specific to the workspace and robot setup at hand.
constraint
  if not using_cumulative_per_zone then
   true
  else
    forall(z in ZONES)(
      cumulative(
          [arrival_time[t] | t in ACTUAL_TASKS]  ++
          [start_time[t] | t in ACTUAL_TASKS] ++
          [end_time[t] | t in ACTUAL_TASKS] ++
          [arrival_time[t] + period | t in ACTUAL_TASKS] ++
          [start_time[t] + period | t in ACTUAL_TASKS] ++
          [end_time[t] + period | t in ACTUAL_TASKS]
          ,
          [ waiting_time[t] | t in ACTUAL_TASKS] ++
          [ duration[t] | t in ACTUAL_TASKS] ++
          [ travel_time[t] | t in ACTUAL_TASKS] ++
          [ waiting_time[t] | t in ACTUAL_TASKS] ++
          [ duration[t] | t in ACTUAL_TASKS] ++
          [ travel_time[t] | t in ACTUAL_TASKS]
          ,
          [wait_blocker[t,z] | t in ACTUAL_TASKS] ++
          [task_blocker[t,z] | t in ACTUAL_TASKS] ++
          [travel_blocker[t,z] | t in ACTUAL_TASKS] ++
          [wait_blocker[t,z] | t in ACTUAL_TASKS] ++
          [task_blocker[t,z] | t in ACTUAL_TASKS] ++
          [travel_blocker[t,z] | t in ACTUAL_TASKS]
          ,
          1
        ) :: domain
    )
  endif
;

% Unified diffn formulation
constraint
  if not using_unified_diffn then
   true
  else
    diffn(
          [arrival_time[t] | t in ACTUAL_TASKS, l in ZONES]  ++
          [start_time[t] | t in ACTUAL_TASKS, l in ZONES] ++
          [end_time[t] | t in ACTUAL_TASKS, l in ZONES] ++
          [arrival_time[t] + period | t in ACTUAL_TASKS, l in ZONES] ++
          [start_time[t] + period | t in ACTUAL_TASKS, l in ZONES] ++
          [end_time[t] + period | t in ACTUAL_TASKS, l in ZONES]
          ,
          [l-1 | n in ACTUAL_TASKS, l in ZONES] ++
          [l-1 | n in ACTUAL_TASKS, l in ZONES] ++
          [l-1 | n in ACTUAL_TASKS, l in ZONES] ++
          [l-1 | n in ACTUAL_TASKS, l in ZONES] ++
          [l-1 | n in ACTUAL_TASKS, l in ZONES] ++
          [l-1 | n in ACTUAL_TASKS, l in ZONES]
          ,
          [ waiting_time[t] | t in ACTUAL_TASKS, l in ZONES] ++
          [ duration[t] | t in ACTUAL_TASKS, l in ZONES] ++
          [ travel_time[t] | t in ACTUAL_TASKS, l in ZONES] ++
          [ waiting_time[t] | t in ACTUAL_TASKS, l in ZONES] ++
          [ duration[t] | t in ACTUAL_TASKS, l in ZONES] ++
          [ travel_time[t] | t in ACTUAL_TASKS, l in ZONES]
          ,
          [wait_blocker[t,z] | t in ACTUAL_TASKS, z in ZONES] ++
          [task_blocker[t,z] | t in ACTUAL_TASKS, z in ZONES] ++
          [travel_blocker[t,z] | t in ACTUAL_TASKS, z in ZONES] ++
          [wait_blocker[t,z] | t in ACTUAL_TASKS, z in ZONES] ++
          [task_blocker[t,z] | t in ACTUAL_TASKS, z in ZONES] ++
          [travel_blocker[t,z] | t in ACTUAL_TASKS, z in ZONES]
        )
  endif
;


% Per zone diffn formulation
constraint
  if not using_diffn_per_zone then
   true
  else
   forall(z in ZONES)(
    diffn(
        [arrival_time[t] | t in ACTUAL_TASKS]  ++
        [start_time[t] | t in ACTUAL_TASKS] ++
        [end_time[t] | t in ACTUAL_TASKS] ++
        [arrival_time[t] + period | t in ACTUAL_TASKS] ++
        [start_time[t] + period | t in ACTUAL_TASKS] ++
        [end_time[t] + period | t in ACTUAL_TASKS]

        ,
        [0 | t in ACTUAL_TASKS] ++
        [0 | t in ACTUAL_TASKS] ++
        [0 | t in ACTUAL_TASKS] ++
        [0 | t in ACTUAL_TASKS] ++
        [0 | t in ACTUAL_TASKS] ++
        [0 | t in ACTUAL_TASKS]
        ,
        [ waiting_time[t] | t in ACTUAL_TASKS] ++
        [ duration[t] | t in ACTUAL_TASKS] ++
        [ travel_time[t] | t in ACTUAL_TASKS] ++
        [ waiting_time[t] | t in ACTUAL_TASKS] ++
        [ duration[t] | t in ACTUAL_TASKS] ++
        [ travel_time[t] | t in ACTUAL_TASKS]
        ,
        [wait_blocker[t,z] | t in ACTUAL_TASKS] ++
        [task_blocker[t,z] | t in ACTUAL_TASKS] ++
        [travel_blocker[t,z] | t in ACTUAL_TASKS] ++
        [wait_blocker[t,z] | t in ACTUAL_TASKS] ++
        [task_blocker[t,z] | t in ACTUAL_TASKS] ++
        [travel_blocker[t,z] | t in ACTUAL_TASKS]
      ) :: domain
    )
  endif
;
