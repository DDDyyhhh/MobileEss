import {expect,test} from 'vitest';
import {mapCenter,screenToWorld} from './MapCanvas';
test('map yaw and nonzero origin are preserved',()=>{
  expect(mapCenter({width:100,height:50,resolution:.1,origin:[2,3,Math.PI/2],data:[],revision:''})[0]).toBeCloseTo(-.5);
  expect(mapCenter({width:100,height:50,resolution:.1,origin:[2,3,Math.PI/2],data:[],revision:''})[1]).toBeCloseTo(8);
});
test('screen Y is inverted once when choosing a goal',()=>{
  expect(screenToWorld(500,200,800,600,[1,2],100)).toEqual([2,3]);
});
