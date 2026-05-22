#!/usr/bin/env python3
"""
circuit_gen_core.py — ARUS Andalucia Racing Team v2

Fixes:
  - Unique session seeds (timestamp) → never repeats circuits
  - Origin alignment: start (0,0) pointing +X → ARUSSim OK
  - DS1 / FS-Rules 2026 v1.1
"""
import json, math, random, sys, time
from pathlib import Path
import numpy as np
from scipy.interpolate import splprep, splev

_ARUS_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(_ARUS_DIR))

try:    from points_to_circuit import profiles as _arus_profiles; _HAS_PROFILES = True
except: _arus_profiles = None;                                     _HAS_PROFILES = False

try:    import mapFile as _arus_mapFile; _HAS_MAPFILE = True
except: _arus_mapFile = None;           _HAS_MAPFILE = False

RULE_LENGTH_MIN   = 200.0
RULE_LENGTH_MAX   = 500.0
RULE_WIDTH_MIN    = 3.0
RULE_OUTER_D_MIN  = 9.0
RULE_CONE_GAP_MAX = 5.0

# Unique session seeds — key to never repeat circuits
_SESSION_OFFSET = int(time.time() * 1000) & 0xFFFFFF
_GEN_COUNTER    = 0

def new_session():
    """Call this every time the user presses 'Generate' to guarantee uniqueness."""
    global _SESSION_OFFSET, _GEN_COUNTER
    _SESSION_OFFSET = int(time.time() * 1000) & 0xFFFFFF
    _GEN_COUNTER    = 0

def _next_seed(user_seed: int, index: int) -> int:
    global _GEN_COUNTER
    _GEN_COUNTER += 1
    s = (user_seed * 1_000_003 + index * 99_991 + _SESSION_OFFSET * 7 + _GEN_COUNTER) & 0xFFFFFFFF
    s ^= s >> 16; s = (s * 0x45d9f3b) & 0xFFFFFFFF
    s ^= s >> 16; s = (s * 0x45d9f3b) & 0xFFFFFFFF
    s ^= s >> 16
    return s if s > 0 else s + 1

# Geometry
def _arc_length(pts):
    diff = np.diff(pts, axis=0, append=pts[:1])
    return float(np.sum(np.hypot(diff[:,0], diff[:,1])))

def _outward_normals(pts):
    n = len(pts)
    ip = (np.arange(n)-1)%n; ix = (np.arange(n)+1)%n
    t  = pts[ix] - pts[ip]
    nm = np.column_stack([t[:,1], -t[:,0]])
    l  = np.hypot(nm[:,0], nm[:,1]); l = np.where(l<1e-10, 1.0, l)
    return nm / l[:,None]

def _offset(pts, norms, d):      return pts + norms * d

def _place_cones(pts, spacing):
    cones=[pts[0].copy()]; acc=0.0
    for i in range(1,len(pts)):
        acc += np.linalg.norm(pts[i]-pts[i-1])
        if acc >= spacing: cones.append(pts[i].copy()); acc=0.0
    return np.array(cones)

def _menger_min_r(cones):
    n=len(cones)
    if n<3: return 9999.0
    mr=9999.0
    for i in range(n):
        p1=cones[(i-1)%n]; p2=cones[i]; p3=cones[(i+1)%n]
        a=np.linalg.norm(p2-p1); b=np.linalg.norm(p3-p2); c=np.linalg.norm(p3-p1)
        area=abs((p2[0]-p1[0])*(p3[1]-p1[1])-(p3[0]-p1[0])*(p2[1]-p1[1]))/2
        d=a*b*c
        if area<1e-8 or d<1e-8: continue
        k=4*area/d
        if k>0.05: mr=min(mr, 1.0/k)
    return mr

def _max_gap(cones):
    if len(cones)<2: return 0.0
    d=np.diff(cones,axis=0,append=cones[:1])
    return float(np.max(np.hypot(d[:,0],d[:,1])))

def _min_width(blue, yellow):
    if not len(blue) or not len(yellow): return 0.0
    diff=blue[:,None,:]-yellow[None,:,:]
    return float(np.min(np.linalg.norm(diff,axis=2)))

# Align to origin → ARUSSim car fix
def _align_to_origin(circuit):
    """
    Move and rotate the circuit so that:
      · The first centerline point is (0, 0)
      · The initial direction is the +X axis
    The ARUSSim car spawns at (0,0) facing +X → lands right at the start.
    """
    cl    = circuit['center']
    start = cl[0].copy()
    idx   = min(60, len(cl)-1)
    dx    = cl[idx,0]-cl[0,0]; dy = cl[idx,1]-cl[0,1]
    if abs(dx)<1e-8 and abs(dy)<1e-8: dx,dy = 1.0, 0.0
    ang   = math.atan2(dy, dx) + math.pi
    ca, sa = math.cos(-ang), math.sin(-ang)

    def T(pts):
        t = pts - start
        return np.column_stack([t[:,0]*ca - t[:,1]*sa,
                                t[:,0]*sa + t[:,1]*ca])
    for k in ('center','outer','inner','blue','yellow'):
        circuit[k] = T(circuit[k])
    return circuit

# Generation
def generate_circuit(seed, target_length=None, track_width=None,
                     n_ctrl_pts=None, cone_spacing=None,
                     max_attempts=120, align_origin=True):
    best = None
    for attempt in range(max_attempts):
        s  = (seed * 1031 + attempt * 37 + 7) & 0xFFFFFFFF
        rg = random.Random(s)
        nr = np.random.RandomState(s)

        npts    = n_ctrl_pts  if n_ctrl_pts  else rg.randint(5, 14)
        half_w  = (track_width/2.0) if track_width else rg.uniform(1.6, 2.6)
        tgt_len = target_length if target_length else rg.uniform(220, 480)
        spacing = cone_spacing  if cone_spacing  else rg.uniform(2.8, 4.7)

        angles = np.sort(nr.uniform(0, 2*math.pi, npts))
        r_base = nr.uniform(10, 75, npts)
        squish_x = nr.uniform(0.25, 1.0, npts)
        squish_y = nr.uniform(0.25, 1.0, npts)
        r_perturb = nr.uniform(0.4, 1.8, npts)
        cx = r_base * r_perturb * np.cos(angles) * squish_x
        cy = r_base * r_perturb * np.sin(angles) * squish_y

        try:
            tck, _ = splprep([cx, cy], s=1.0, per=1, k=3)
        except:
            continue

        u = np.linspace(0, 1, 5000, endpoint=False)
        xs, ys = splev(u, tck)
        center = np.column_stack([xs, ys])
        raw_l  = _arc_length(center)
        if raw_l < 1.0: continue
        center *= tgt_len / raw_l

        norms  = _outward_normals(center)
        outer  = _offset(center, norms,  half_w)
        inner  = _offset(center, norms, -half_w)
        blue   = _place_cones(outer, spacing)
        yellow = _place_cones(inner, spacing)
        if len(blue)<4 or len(yellow)<4: continue

        tl  = _arc_length(center)
        mw  = _min_width(blue, yellow)
        mrd = 2.0 * _menger_min_r(blue)
        cg  = max(_max_gap(blue), _max_gap(yellow))

        rules = {
            'length':   {'label':'Track length',    'req':'200 – 500 m','value':tl,  'fmt':f'{tl:.1f} m',  'pass':RULE_LENGTH_MIN<=tl<=RULE_LENGTH_MAX},
            'width':    {'label':'Min track width', 'req':'>= 3 m',     'value':mw,  'fmt':f'{mw:.2f} m',  'pass':mw>=RULE_WIDTH_MIN},
            'outer_d':  {'label':'Min outer diam.', 'req':'>= 9 m',     'value':mrd, 'fmt':f'{mrd:.2f} m', 'pass':mrd>=RULE_OUTER_D_MIN},
            'cone_gap': {'label':'Max cone gap',    'req':'<= 5 m',     'value':cg,  'fmt':f'{cg:.2f} m',  'pass':cg<=RULE_CONE_GAP_MAX},
        }
        ok = all(r['pass'] for r in rules.values())
        res = {
            'seed':seed, 'attempt':attempt, 'compliant':ok,
            'center':center,'outer':outer,'inner':inner,'blue':blue,'yellow':yellow,
            'rules':rules,
            'params':{'length':tl,'width':2*half_w,'spacing':spacing,'n_ctrl':npts,
                      'n_blue':len(blue),'n_yellow':len(yellow)},
        }
        if ok:
            return _align_to_origin(res) if align_origin else res
        if best is None: best = res

    if best:
        return _align_to_origin(best) if align_origin else best
    return _align_to_origin(res) if align_origin else res

def generate_batch(count, base_seed=2026, **kw):
    return [generate_circuit(_next_seed(base_seed, i), **kw) for i in range(count)]

# Speed profile
def _speed_profile(center):
    if _HAS_PROFILES:
        return _arus_profiles(center[:,0].tolist(), center[:,1].tolist())
    n=len(center); s=[0.0]; xp,yp=[],[]
    for i in range(n-1):
        dx=center[i+1,0]-center[i,0]; dy=center[i+1,1]-center[i,1]
        dist=math.hypot(dx,dy); xp.append(dx); yp.append(dy); s.append(s[-1]+dist)
    xp.append(xp[-1]); yp.append(yp[-1])
    xpp=[xp[i+1]-xp[i] for i in range(n-1)]+[xp[-1]-xp[-2] if n>1 else 0]
    ypp=[yp[i+1]-yp[i] for i in range(n-1)]+[yp[-1]-yp[-2] if n>1 else 0]
    vm,am,ay=15.0,5.0,4.0
    k=[((xp[i]*ypp[i]-xpp[i]*yp[i])/max((xp[i]**2+yp[i]**2)**1.5,1e-8)) for i in range(n)]
    vg=[min(math.sqrt(ay/max(abs(ki),1e-4)),vm) for ki in k]
    sp=[0.0]*n; sp[0]=1.0
    for i in range(1,n): sp[i]=min(math.sqrt(sp[i-1]**2+2*am*(s[i]-s[i-1])),vg[i])
    for i in range(n-2,-1,-1):
        vb=math.sqrt(sp[i+1]**2+2*am*(s[i+1]-s[i]))
        if sp[i]>vb: sp[i]=vb
    ac=[0.0]*n
    for i in range(n-1):
        ds=s[i+1]-s[i]
        if ds>1e-8: ac[i]=(sp[i+1]**2-sp[i]**2)/(2*ds)
    return {'x':center[:,0].tolist(),'y':center[:,1].tolist(),'s':s,'k':k,'speed_profile':sp,'acc_profile':ac}

# Export
def export_pcd(circuit, path):
    b=circuit['blue']; y=circuit['yellow']
    bo=np.array([circuit['outer'][0],circuit['inner'][0]])
    all_c=([(p[0],p[1],0) for p in b]+[(p[0],p[1],1) for p in y]+[(p[0],p[1],3) for p in bo])
    hdr=(f"# .PCD v0.7 - Point Cloud Data file format\n"
         f"# ARUS FSG 2026 Circuit Generator v2  seed={circuit['seed']}\n"
         f"# aligned: start=(0,0) heading=-X\n"
         f"VERSION 0.7\nFIELDS x y z color score\nSIZE 4 4 4 4 4\nTYPE F F F I F\n"
         f"COUNT 1 1 1 1 1\nWIDTH {len(all_c)}\nHEIGHT 1\n"
         f"VIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(all_c)}\nDATA ascii\n")
    rows="\n".join(f"{x:.6f} {y:.6f} 0.000000 {t} 1" for x,y,t in all_c)
    Path(path).write_text(hdr+rows+"\n")

def export_json(circuit, path):
    Path(path).write_text(json.dumps(_speed_profile(circuit['center']), indent=2))

def export_both(circuit, base):
    export_pcd(circuit, base+'.pcd'); export_json(circuit, base+'.json')

# CLI
if __name__ == '__main__':
    import argparse
    ap=argparse.ArgumentParser()
    ap.add_argument('-n','--count',  type=int,   default=5)
    ap.add_argument('-s','--seed',   type=int,   default=2026)
    ap.add_argument('-o','--out',    type=str,   default='.')
    ap.add_argument('-l','--length', type=float, default=None)
    ap.add_argument('-w','--width',  type=float, default=None)
    ap.add_argument('--spacing',     type=float, default=None)
    ap.add_argument('--all',         action='store_true')
    args=ap.parse_args()
    out=Path(args.out); out.mkdir(parents=True,exist_ok=True)
    new_session(); ok=0
    print(f"\n[ARUS] {args.count} circuits  seed={args.seed}  session={_SESSION_OFFSET}\n")
    for i in range(args.count):
        seed=_next_seed(args.seed,i)
        c=generate_circuit(seed,args.length,args.width,None,args.spacing)
        st='✓' if c['compliant'] else '✗'
        print(f"  [{st}] {seed:010d}  L={c['params']['length']:.0f}m  cones={c['params']['n_blue']+c['params']['n_yellow']}  att={c['attempt']}")
        if c['compliant']: ok+=1
        if c['compliant'] or args.all: export_both(c,str(out/f"circuit_{seed:010d}"))
    print(f"\n[ARUS] {ok}/{args.count} compliant → {out.resolve()}\n")
