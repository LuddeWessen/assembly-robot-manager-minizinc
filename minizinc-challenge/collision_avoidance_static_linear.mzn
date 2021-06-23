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


% Constraint 8 of CPAIOR-paper (As suggested by reviewer)
%agent[n], location[n], max_PL_left_arm
%Only post for non-fixture locations:
%0, x, y | x < y
%1, x, y | x >= y

int: max_lo = max(location_order);
var 0..max_lo: left_max;

constraint forall(p in ACTUAL_TASKS) (
  if p in FIXTURE_TASKS then
    true
  else
    let { array[int] of int: ws_lin_separation_raw = [

    if x = 1 then
      1
    elseif x = 2 then
      l1
    else
      location_order[l2]
    endif
    | l1 in location_domain(p) where left_arm_travel_times[l1,l1] >= 0 /\ location_order[l1] <= FixtureWorkObstruction[1] , l2 in LOCATIONS where location_order[l1] <= location_order[l2], x in 1..3]

  %Right arm - accept everything to the right
  ++ [
    if x = 1 then
      2
    elseif x = 2 then
      l1
    else
      location_order[l2]
    endif
    | l1 in location_domain(p) where right_arm_travel_times[l1,l1] >= 0 /\ (location_order[l1] >= FixtureWorkObstruction[2] \/ location_order[l1] < 0), l2 in LOCATIONS where location_order[l1] > location_order[l2], x in 1..3] ;

    } in table( [agent[p]] ++ [location[p]] ++ [left_max], array2d(1..(length(ws_lin_separation_raw) div 3), 1..3, ws_lin_separation_raw))
  endif
);

%Equivalent to above, and about same speed:
%constraint forall(n in ACTUAL_TASKS diff FIXTURE_TASKS)(agent[n] == 1 <-> location_order[location[n]] <= left_max);

%Constraint (8) should enforce that the two arms do not collide when each of them performing a task from non-sharable zones.
% The stated constraint is time-independent when the tasks are executed.
% This seems to be too restricted and may lead to a removal of feasible solutions.
% As is, this constraint will partition the non-sharable locations into two sets,
% each set containing locations that only can be access from one arm.
