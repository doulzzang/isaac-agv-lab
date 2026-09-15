import {env} from 'cloudflare:workers';
import {store,state,online,controlKey,agentAuthorized} from './relay';
const reply=(data:unknown,status=200)=>Response.json(data,{status,headers:{'Cache-Control':'no-store'}});
const key=(id:string)=>'private-isaac-assets/'+id;
export async function assetGet(req:Request){
 if(!await agentAuthorized(req))return reply({error:'운영 PC 인증 실패'},401);
 const id=new URL(req.url).searchParams.get('id')||'';
 if(!/^[a-f0-9-]{36}\.(usd|usda|usdc|usdz|zip)$/.test(id))return reply({error:'잘못된 파일'},400);
 const data=await env.BUCKET.get(key(id));if(!data)return reply({error:'파일 없음'},404);
 return new Response(data.body,{headers:{'Content-Type':'application/octet-stream','Cache-Control':'no-store'}});
}
export async function assetUpload(req:Request){
 if(!online(await state()))return reply({error:'운영 PC 연결 필요'},503);
 const db=await store(),holder=await controlKey(req),now=Date.now();
 const held=await db.prepare('SELECT id FROM isaac_control WHERE holder=? AND holder<>\'\' AND expires>?').bind(holder,now).first();
 if(!held)return reply({error:'조작권을 먼저 요청하세요.'},403);
 const kind=req.headers.get('x-asset-kind')||'room';
 if(!['room','object'].includes(kind))return reply({error:'잘못된 파일 용도'},400);
 const name=decodeURIComponent(req.headers.get('x-file-name')||'');
 const extension=name.split('.').pop()?.toLowerCase();
 if(!['usd','usda','usdc','usdz','zip'].includes(extension||''))return reply({error:'USD, USDA, USDC, USDZ 또는 ZIP 파일을 선택하세요.'},400);
 const bytes=await req.arrayBuffer();if(!bytes.byteLength||bytes.byteLength>20*1024*1024)return reply({error:'20MB 이하 파일을 선택하세요.'},413);
 const id=crypto.randomUUID()+'.'+extension;
 await env.BUCKET.put(key(id),bytes,{customMetadata:{uploaded:String(now)}});
 const value={id,kind,label:name.replace(/\.[^.]+$/,'').slice(0,80)};
 const result=await db.prepare("INSERT INTO isaac_relay_commands(id,action,value,created,expires) SELECT ?,'asset_import',?,?,? WHERE EXISTS(SELECT 1 FROM isaac_control WHERE holder=? AND expires>?) RETURNING id").bind(crypto.randomUUID(),JSON.stringify(value),now,now+60000,holder,Date.now()).first();
 if(!result){await env.BUCKET.delete(key(id));return reply({error:'조작권이 만료되었습니다. 다시 요청하세요.'},403);}
 return reply({ok:true,id,message:'운영 PC로 전달 중입니다. 목록에 나타나면 선택하세요.'},202);
}
