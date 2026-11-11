import {expect,test} from 'vitest';
import {mapCenter,screenToWorld} from './MapCanvas';
test('map yaw and nonzero origin are preserved',()=>{
  expect(mapCenter({width:100,height:50,resolution:.1,origin:[2,3,Math.PI/2],data:[],revision:''})[0]).toBeCloseTo(-.5);
  expect(mapCenter({width:100,height:50,resolution:.1,origin:[2,3,Math.PI/2],data:[],revision:''})[1]).toBeCloseTo(8);
});
test('screen Y is inverted once when choosing a goal',()=>{
  expect(screenToWorld(500,200,800,600,[1,2],100)).toEqual([2,3]);
});
import {goalArray,goalFromArray,headingFromDrag,sameGoal,unacceptedKinds} from './types';
test('dragging from the goal sets a map-frame heading, ignoring tiny drags',()=>{
  const goal={x:1,y:2,yaw:null};
  expect(headingFromDrag(goal,[1,3])).toBeCloseTo(Math.PI/2);
  expect(headingFromDrag(goal,[0,2])).toBeCloseTo(Math.PI);
  expect(headingFromDrag(goal,[1.05,2.05])).toBeNull();
});
test('goal heading is sent only when chosen and compared exactly with the preview',()=>{
  expect(goalArray({x:1,y:2,yaw:null})).toEqual([1,2]);
  expect(goalArray({x:1,y:2,yaw:.5})).toEqual([1,2,.5]);
  expect(goalFromArray([1,2,.5])).toEqual({x:1,y:2,yaw:.5});
  expect(sameGoal([1,2],{x:1,y:2,yaw:null})).toBe(true);
  expect(sameGoal([1,2],{x:1,y:2,yaw:.5})).toBe(false);
});
test('routes needing unaccepted motion are reported once per kind',()=>{
  const seg=(kind:'forward'|'rotate'|'reverse')=>({kind,poses:[],length_m:0,turn:0});
  expect(unacceptedKinds([seg('forward'),seg('rotate'),seg('forward')],{forward:true,rotate:false})).toEqual(['rotate']);
  expect(unacceptedKinds([seg('forward')],undefined)).toEqual(['forward']);
});
