import { env } from 'cloudflare:workers';

export type RelayState = { token_hash: string; agent_id: string; seen: number; status: string; viewer_seen: number };
export type Command = { id: string; action: string; value: string; created: number; expires: number };
let initialized: Promise<unknown> | undefined;
export async function store() {
  initialized ??= env.DB.batch([
    env.DB.prepare(`CREATE TABLE IF NOT EXISTS isaac_relay (id INTEGER PRIMARY KEY CHECK(id=1), token_hash TEXT NOT NULL, agent_id TEXT NOT NULL DEFAULT '', seen INTEGER NOT NULL DEFAULT 0, status TEXT NOT NULL DEFAULT '{}', viewer_seen INTEGER NOT NULL DEFAULT 0)`),
    env.DB.prepare(`CREATE TABLE IF NOT EXISTS isaac_relay_commands (id TEXT PRIMARY KEY, action TEXT NOT NULL, value TEXT NOT NULL, created INTEGER NOT NULL, expires INTEGER NOT NULL, claimed INTEGER NOT NULL DEFAULT 0)`),
    env.DB.prepare(`CREATE TABLE IF NOT EXISTS isaac_control (id INTEGER PRIMARY KEY CHECK(id=1), holder TEXT NOT NULL DEFAULT '', expires INTEGER NOT NULL DEFAULT 0)`),
    env.DB.prepare(`INSERT OR IGNORE INTO isaac_control(id) VALUES(1)`),
    env.DB.prepare(`CREATE TABLE IF NOT EXISTS isaac_pairing (code TEXT PRIMARY KEY, token_hash TEXT NOT NULL, expires INTEGER NOT NULL)`),
    env.DB.prepare(`CREATE TABLE IF NOT EXISTS isaac_visitors(id TEXT PRIMARY KEY,approved INTEGER NOT NULL,seen INTEGER NOT NULL)`),
  ]).catch((e: unknown)=>{initialized=undefined;throw e;});
  await initialized;
  return env.DB;
}
export async function state(): Promise<RelayState | null> {
  return (await store()).prepare('SELECT * FROM isaac_relay WHERE id=1').first<RelayState>();
}
export async function digest(value: string) {
  return Array.from(new Uint8Array(await crypto.subtle.digest('SHA-256',new TextEncoder().encode(value))),b=>b.toString(16).padStart(2,'0')).join('');
}
export async function agentAuthorized(request: Request) {
  const token=request.headers.get('authorization')?.replace(/^Bearer /,'')||'';
  if(token.length<32 || token.length>256)return false;
  const row=await state();
  return !!row && (await digest(token))===row.token_hash;
}
export const ALLOWED = new Set([
  'message_ack','navigation_select','navigation_refresh','navigation_goal',
  'room_change','room_reset','room_discard','room_save','room_load','room_refresh','object_select','object_add','object_apply','object_delete',
  'scene_minimal','scene_warehouse','add_agv','worker_start','worker_stop','worker_reset','worker_rotate','worker_smaller','worker_larger',
  'humidity_prepare','humidity_up','humidity_down','temperature_prepare','temperature_up','temperature_down',
  'robot_forward','robot_backward','robot_left','robot_right','robot_release','robot_stop','robot_reset','robot_resize_default','robot_body_red',
  'prepare_next_experience','run_experience','agv_camera','close_camera','view_overview','view_pan_left','view_pan_right','view_pan_up','view_pan_down',
  'view_zoom_in','view_zoom_out','view_restore','fixed','movable','kinematic','mass','smaller','larger','color','model','agent_prompt','face_map_2d',
]);
export const MANUAL = new Set(['robot_forward','robot_backward','robot_left','robot_right']);
export function online(row: RelayState | null, now=Date.now()) {return !!row && now-row.seen<10000;}
export function validateCommand(data: unknown): {action: string; value: unknown} {
  if(!data || typeof data!=='object')throw new Error('명령 형식이 올바르지 않습니다.');
  const {action,value} = data as {action: string;value: unknown};
  if(!ALLOWED.has(action))throw new Error('홈페이지에서 지원하지 않는 명령입니다.');
  if(JSON.stringify(value??null).length>6000)throw new Error('명령이 너무 큽니다.');
  return {action,value:value??null};
}

export async function controlKey(req: Request) {
  const token=req.headers.get('x-isaac-control')||'';
  return /^[a-zA-Z0-9-]{32,80}$/.test(token)?digest(token):'';
}
export async function expireControl(now=Date.now()) {
  const db=await store();
  await db.batch([
    db.prepare("DELETE FROM isaac_relay_commands WHERE claimed=0 AND EXISTS(SELECT 1 FROM isaac_control WHERE holder<>'' AND expires<=?)").bind(now),
    db.prepare("INSERT INTO isaac_relay_commands(id,action,value,created,expires) SELECT ?, 'robot_stop','null',?,? FROM isaac_control WHERE holder<>'' AND expires<=?").bind(crypto.randomUUID(),now,now+5000,now),
    db.prepare("UPDATE isaac_control SET holder='',expires=0 WHERE expires<=?").bind(now),
  ]);
}
export async function controlStatus(key: string) {
  const row=await (await store()).prepare('SELECT holder,expires FROM isaac_control WHERE id=1').first<{holder:string;expires:number}>();
  const held=!!row?.holder && row.expires>Date.now();
  return {held,mine:held&&row!.holder===key,operator:held?'접속 '+row!.holder.slice(0,6):null,expires:held?row!.expires:0};
}
