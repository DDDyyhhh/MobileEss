import {ButtonHTMLAttributes,ReactNode,useEffect,useId,useRef} from 'react';

export type IconName='map'|'layers'|'target'|'plus'|'minus'|'expand'|'arrow'|'chevron'|'close'|'stop'|'info'|'check'|'lock'|'wifi'|'cube'|'rotate'|'route'|'pin'|'alert';
const paths:Record<IconName,ReactNode>={
  map:<><path d="m3 5 6-2 6 2 6-2v16l-6 2-6-2-6 2zM9 3v16m6-14v16"/></>,
  layers:<><path d="m12 3 9 5-9 5-9-5zM3 12l9 5 9-5M3 16l9 5 9-5"/></>,
  target:<><circle cx="12" cy="12" r="7"/><circle cx="12" cy="12" r="2"/><path d="M12 2v3m0 14v3M2 12h3m14 0h3"/></>,
  plus:<path d="M12 5v14M5 12h14"/>,minus:<path d="M5 12h14"/>,
  expand:<path d="M8 3H3v5m13-5h5v5M3 16v5h5m8 0h5v-5"/>,
  arrow:<path d="M4 12h16m-6-6 6 6-6 6"/>,chevron:<path d="m8 5 7 7-7 7"/>,
  close:<path d="m6 6 12 12M6 18 18 6"/>,stop:<rect x="5" y="5" width="14" height="14" rx="3"/>,
  info:<><circle cx="12" cy="12" r="9"/><path d="M12 11v6m0-10v.1"/></>,
  check:<path d="m5 12 4 4L19 6"/>,lock:<><rect x="5" y="10" width="14" height="11" rx="3"/><path d="M8 10V7a4 4 0 0 1 8 0v3m-4 5v2"/></>,
  wifi:<><path d="M2 8a16 16 0 0 1 20 0M5 12a11 11 0 0 1 14 0m-11 4a6 6 0 0 1 8 0"/><circle cx="12" cy="20" r=".6"/></>,
  cube:<><path d="m12 2 9 5v10l-9 5-9-5V7zM3 7l9 5 9-5m-9 5v10M7.5 4.5l9 5"/></>,
  rotate:<><path d="M20 8a8 8 0 1 0 0 8M20 3v5h-5"/></>,
  route:<><circle cx="5" cy="5" r="2"/><circle cx="19" cy="19" r="2"/><path d="M7 5h9a4 4 0 0 1 0 8H8a3 3 0 0 0 0 6h9"/></>,
  pin:<><path d="M19 10c0 5-7 12-7 12S5 15 5 10a7 7 0 1 1 14 0z"/><circle cx="12" cy="10" r="2"/></>,
  alert:<><path d="M10.3 3.8 2.5 18a2 2 0 0 0 1.8 3h15.4a2 2 0 0 0 1.8-3L13.7 3.8a2 2 0 0 0-3.4 0Z"/><path d="M12 9v5m0 3v.1"/></>,
};
export function Icon({name,size=20}:{name:IconName;size?:number}) {
  return <svg width={size} height={size} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.6" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">{paths[name]}</svg>;
}
export function Button({variant='primary',className='',children,...props}:ButtonHTMLAttributes<HTMLButtonElement>&{variant?:'primary'|'secondary'|'quiet'|'stop';children:ReactNode}) {
  return <button type="button" className={`button button-${variant} ${className}`} {...props}>{children}</button>;
}
export function IconButton({icon,label,...props}:{icon:IconName;label:string}&ButtonHTMLAttributes<HTMLButtonElement>) {
  return <button type="button" className="icon-button" title={label} aria-label={label} {...props}><Icon name={icon}/></button>;
}
export function Modal({title,children,onClose,drawer=false}:{title:string;children:ReactNode;onClose?:()=>void;drawer?:boolean}) {
  const ref=useRef<HTMLDivElement>(null),id=useId(),close=useRef(onClose);close.current=onClose;
  useEffect(()=>{
    const previous=document.activeElement instanceof HTMLElement?document.activeElement:null;
    const root=ref.current!;
    const focusable=()=>Array.from(root.querySelectorAll<HTMLElement>('button:not([disabled]),input:not([disabled]),select:not([disabled]),a[href],[tabindex="0"]')).filter(el=>el.getAttribute('aria-hidden')!=='true');
    (root.querySelector<HTMLElement>('[data-autofocus]')??focusable()[0]??root).focus();
    const key=(event:KeyboardEvent)=>{
      if(event.key==='Escape'&&close.current){event.preventDefault();close.current();}
      if(event.key==='Tab'){
        const items=focusable(),first=items[0],last=items.at(-1);
        if(!first){event.preventDefault();root.focus();}
        else if(event.shiftKey&&(document.activeElement===first||document.activeElement===root)){event.preventDefault();last?.focus();}
        else if(!event.shiftKey&&document.activeElement===last){event.preventDefault();first.focus();}
      }
    };
    root.addEventListener('keydown',key);
    return()=>{root.removeEventListener('keydown',key);previous?.focus();};
  },[]);
  return <div className={`modal-backdrop ${drawer?'drawer-backdrop':''}`} onPointerDown={e=>{if(e.target===e.currentTarget)onClose?.();}}>
    <div ref={ref} tabIndex={-1} className={`modal ${drawer?'drawer':''}`} role="dialog" aria-modal="true" aria-labelledby={id}>
      <div className="modal-heading"><h2 id={id}>{title}</h2>{onClose&&<IconButton icon="close" label="关闭对话框" onClick={onClose}/>}</div>
      {children}
    </div>
  </div>;
}
