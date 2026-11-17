/** Shared display tokens. Geometry and map coordinates are always in metres. */
export const mapTheme = {
  canvas: '#f5f5f7', free: '#ffffff', unknown: '#e5e5ea', occupied: '#636366',
  grid: '#e0e0e5', ink: '#1d1d1f', muted: '#86868b',
  blue: '#0066cc', reverse: '#537a9e', rotate: '#1d1d1f',
  obstacle: '#b42318', history: '#86868b', clearance: '#0066cc12',
  dark: '#252527', darkGrid: '#38383b', onDark: '#f5f5f7', live: '#2997ff',
} as const;
export type Layers = {route: boolean; trajectory: boolean; obstacles: boolean; cloud: boolean; staticCloud: boolean};
export const defaultLayers: Layers = {route:true, trajectory:true, obstacles:true, cloud:true, staticCloud:true};
export type ViewCommand = {kind:'fit'|'vehicle'|'in'|'out'; serial:number};
