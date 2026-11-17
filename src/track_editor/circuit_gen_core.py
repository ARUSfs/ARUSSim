#!/usr/bin/env python3
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
    d=np.diff(pts,axis=0,append=pts[:1])
    return float(np.sum(np.hypot(d[:,0],d[:,1])))

def _outward_normals(pts):
    n = len(pts)
    ip = (np.arange(n)-1)%n; ix = (np.arange(n)+1)%n
    t  = pts[ix] - pts[ip]
    nm = np.column_stack([t[:,1], -t[:,0]])
    l  = np.hypot(nm[:,0], nm[:,1]); l = np.where(l<1e-10, 1.0, l)
    return nm / l[:,None]

def _offset(pts, norms, d):      return pts + norms * d

def _place_cones_adaptive(pts, spacing, min_spacing=1.5):
    """Place cones every `spacing` m, tightened in curves to 0.5*local radius (>= min_spacing)."""
    n=len(pts)
    ip=(np.arange(n)-8)%n; ix=(np.arange(n)+8)%n
    a=pts[ip]; b=pts; c=pts[ix]
    ab=np.linalg.norm(b-a,axis=1); bc=np.linalg.norm(c-b,axis=1); ac=np.linalg.norm(c-a,axis=1)
    area=np.abs((b[:,0]-a[:,0])*(c[:,1]-a[:,1])-(c[:,0]-a[:,0])*(b[:,1]-a[:,1]))/2
    radius=np.where(area>1e-9, ab*bc*ac/np.maximum(4*area,1e-12), 1e9)
    local=np.clip(0.5*radius, min_spacing, spacing)
    cones=[pts[0].copy()]; acc=0.0
    for i in range(1,n):
        acc += np.linalg.norm(pts[i]-pts[i-1])
        if acc >= local[i]: cones.append(pts[i].copy()); acc=0.0
    return np.array(cones)

def _resample(pts, step):
    """Closed polyline resampled to roughly constant spacing `step` (m)."""
    d=np.hypot(*np.diff(pts,axis=0,append=pts[:1]).T)
    s=np.concatenate([[0],np.cumsum(d)])
    t=np.arange(0,s[-1],step)
    closed=np.vstack([pts,pts[:1]])
    return np.column_stack([np.interp(t,s,closed[:,0]),np.interp(t,s,closed[:,1])])

def _center_min_r(center, base=1.5):
    """Minimum centerline turning radius (m), measured over chords of ~`base` m."""
    p=_resample(center,0.5); k=max(1,int(round(base/0.5)))
    a=np.roll(p,k,axis=0); c=np.roll(p,-k,axis=0)
    ab=np.linalg.norm(p-a,axis=1); bc=np.linalg.norm(c-p,axis=1); ac=np.linalg.norm(c-a,axis=1)
    area=np.abs((p[:,0]-a[:,0])*(c[:,1]-a[:,1])-(c[:,0]-a[:,0])*(p[:,1]-a[:,1]))/2
    r=np.where(area>1e-9, ab*bc*ac/np.maximum(4*area,1e-12), 1e9)
    return float(r.min())

def _min_clearance(center, min_arc):
    """Min distance between centerline points that are more than `min_arc` m apart along the track."""
    p=_resample(center,1.0); n=len(p)
    d=np.linalg.norm(p[:,None,:]-p[None,:,:],axis=2)
    i=np.arange(n); sep=np.abs(i[:,None]-i[None,:]); sep=np.minimum(sep,n-sep)*1.0
    d[sep<=min_arc]=np.inf
    return float(d.min()) if np.isfinite(d).any() else 9999.0

def _max_gap(cones):
    if len(cones)<2: return 0.0
    d=np.diff(cones,axis=0,append=cones[:1])
    return float(np.max(np.hypot(d[:,0],d[:,1])))

def _min_width(blue, yellow):
    if not len(blue) or not len(yellow): return 0.0
    diff=blue[:,None,:]-yellow[None,:,:]
    return float(np.min(np.linalg.norm(diff,axis=2)))

def _polar_to_xy(r, angles, rot=0.0):
    """Converts (r, angles) to (cx, cy) with optional rotation."""
    ang = angles + rot
    return r*np.cos(ang), r*np.sin(ang)

def _shape_oval(rg, nr, hw):
    """Oval with variable aspect ratio and rotation."""
    n   = rg.randint(7, 14)
    R   = rg.uniform(20, 70)
    asp = rg.uniform(0.35, 1.0)   # aspect ratio b/a
    rot = rg.uniform(0, math.pi)
    ang = np.linspace(0, 2*math.pi, n, endpoint=False)
    ang += nr.uniform(-0.15, 0.15, n)
    a = R; b = R * asp
    r = a*b / np.sqrt((b*np.cos(ang))**2 + (a*np.sin(ang))**2)
    r *= nr.uniform(0.88, 1.12, n)
    return _polar_to_xy(r, ang, rot)

def _shape_hairpin(rg, nr, hw):
    """
    Hairpin: the curve has a very reduced radius in a sector (the hook)
    and a large radius in the opposite sector (the wide turn).
    There are no crossings because everything is polar.
    """
    n       = rg.randint(10, 18)
    R_base  = rg.uniform(20, 60)
    R_min   = rg.uniform(0.12, 0.28) * R_base  # minimum radius at the tip
    hp_width = rg.uniform(0.12, 0.30)           # angular amplitude of the hook (rad)
    hp_angle = rg.uniform(0, 2*math.pi)          # position of the hook
    rot      = rg.uniform(0, 2*math.pi)

    ang = np.linspace(0, 2*math.pi, n, endpoint=False)
    # Function of radius: minimum at hp_angle, maximum at hp_angle+π
    delta = np.abs(np.arctan2(np.sin(ang - hp_angle), np.cos(ang - hp_angle)))
    # delta ∈ [0,π]: 0=tip, π=opposite side
    t = np.clip(1.0 - delta/hp_width, 0, 1)**2  # smooth bell
    r = R_base * (1.0 - (1.0 - R_min/R_base)*t)
    r *= nr.uniform(0.9, 1.1, n)
    ang += nr.uniform(-0.08, 0.08, n)
    return _polar_to_xy(r, ang, rot)

def _shape_chicane(rg, nr, hw):
    """
    Chicana: multiple alternating zones of reduced radius, separated by ~π/n_chi.
    """
    n       = rg.randint(12, 20)
    n_chi   = rg.randint(2, 4)    # number of chicanes
    R_base  = rg.uniform(22, 65)
    R_min   = rg.uniform(0.25, 0.55) * R_base
    chi_w   = rg.uniform(0.08, 0.18)            # angular width of each chicane
    rot     = rg.uniform(0, 2*math.pi)

    ang = np.linspace(0, 2*math.pi, n, endpoint=False)
    r   = np.full(n, R_base)

    for j in range(n_chi):
        center_j = 2*math.pi * j / n_chi + rg.uniform(-0.1, 0.1)
        delta = np.abs(np.arctan2(np.sin(ang-center_j), np.cos(ang-center_j)))
        t = np.clip(1.0-delta/chi_w, 0, 1)**2
        r = r * (1.0 - (1.0 - R_min/R_base)*t)

    # Variable aspect ratio
    asp = rg.uniform(0.45, 0.95)
    a   = R_base; b = R_base*asp
    r_ell = a*b / np.sqrt((b*np.cos(ang))**2+(a*np.sin(ang))**2)
    r = r * (r_ell / R_base)
    r *= nr.uniform(0.92, 1.08, n)
    ang += nr.uniform(-0.06, 0.06, n)
    return _polar_to_xy(r, ang, rot)

def _shape_teardrop(rg, nr, hw):
    """
    Teardrop: minimum radius at θ=0 (tip), maximum at θ=π.
    """
    n      = rg.randint(10, 18)
    R_big  = rg.uniform(22, 65)
    R_tip  = rg.uniform(0.08, 0.22) * R_big
    sharpness = rg.uniform(1.5, 4.0)   # how sharp the tip is
    rot    = rg.uniform(0, 2*math.pi)

    ang = np.linspace(0, 2*math.pi, n, endpoint=False)
    # r(θ) = smooth interpolation between R_tip (θ=0) and R_big (θ=π)
    t = (1 - np.cos(ang)) / 2      # 0 at θ=0, 1 at θ=π, 0.5 at θ=π/2
    t_shaped = t**sharpness
    r = R_tip + (R_big - R_tip) * t_shaped
    r *= nr.uniform(0.92, 1.08, n)
    ang += nr.uniform(-0.06, 0.06, n)
    return _polar_to_xy(r, ang, rot)

def _shape_sshape(rg, nr, hw):
    """
    S: two minimum radii at opposite positions (θ=0 and θ=π),
    two maximum radii at θ=π/2 and θ=3π/2.
    The result is a closed curve in the shape of an S or smoothed figure-8.
    """
    n      = rg.randint(10, 18)
    R_base = rg.uniform(22, 65)
    R_min  = rg.uniform(0.20, 0.50) * R_base
    depth  = rg.uniform(0.6, 1.0)   # depth of the S
    rot    = rg.uniform(0, 2*math.pi)
    asp    = rg.uniform(0.45, 0.90)

    ang = np.linspace(0, 2*math.pi, n, endpoint=False)
    # Two minima at 0 and π via cos(2θ)
    t = (1 + np.cos(2*ang)) / 2   # 1 at 0,π  → minimum; 0 at π/2,3π/2 → maximum
    r = R_base * (1 - depth*(1-R_min/R_base)*t)
    # Add aspect ratio
    a=R_base; b=R_base*asp
    r_ell = a*b/np.sqrt((b*np.cos(ang))**2+(a*np.sin(ang))**2)
    r = r * r_ell / R_base
    r *= nr.uniform(0.92, 1.08, n)
    ang += nr.uniform(-0.06, 0.06, n)
    return _polar_to_xy(r, ang, rot)

def _shape_lshape(rg, nr, hw):
    """
    L: an sector of ~90° has a very large radius (long straight),
    the opposite sector has a normal radius,
    the corner has a reduced radius.
    """
    n       = rg.randint(10, 18)
    R_base  = rg.uniform(22, 60)
    R_corner = rg.uniform(0.15, 0.35) * R_base  # corner
    R_long   = rg.uniform(1.4, 2.8)  * R_base   # long straight (large radius = far from center)
    corner_a = rg.uniform(0, 2*math.pi)
    long_a   = corner_a + math.pi + rg.uniform(-0.5, 0.5)
    w_corner = rg.uniform(0.12, 0.28)
    w_long   = rg.uniform(0.35, 0.70)
    rot      = rg.uniform(0, 2*math.pi)

    ang = np.linspace(0, 2*math.pi, n, endpoint=False)
    r   = np.full(n, R_base, dtype=float)

    # Add corner (local minimum)
    delta_c = np.abs(np.arctan2(np.sin(ang-corner_a), np.cos(ang-corner_a)))
    t_c = np.clip(1 - delta_c/w_corner, 0, 1)**2
    r = r*(1 - (1-R_corner/R_base)*t_c)

    # Add long straight (local maximum)
    delta_l = np.abs(np.arctan2(np.sin(ang-long_a), np.cos(ang-long_a)))
    t_l = np.clip(1 - delta_l/w_long, 0, 1)**2
    r = r*(1 + (R_long/R_base - 1)*t_l)

    r *= nr.uniform(0.92, 1.08, n)
    ang += nr.uniform(-0.06, 0.06, n)
    return _polar_to_xy(r, ang, rot)

def _shape_random_walk(rg, nr, hw):
    """Polar forms with non-uniform distributions → free forms, always closed."""
    n      = rg.randint(8, 18)
    R_base = rg.uniform(22, 70)
    angles = np.cumsum(nr.uniform(0.2, 1.2, n))
    angles = angles/angles[-1]*2*math.pi
    radii  = nr.uniform(0.3, 1.2, n)*R_base
    sx=nr.uniform(0.35,1.6); sy=nr.uniform(0.35,1.6)
    cx=radii*np.cos(angles)*sx+nr.normal(0,R_base*0.07,n)
    cy=radii*np.sin(angles)*sy+nr.normal(0,R_base*0.07,n)
    rot=rg.uniform(0,2*math.pi)
    return cx*math.cos(rot)-cy*math.sin(rot), cx*math.sin(rot)+cy*math.cos(rot)

def _shape_irregular(rg, nr, hw):
    """
    Multiple hairpins in irregular positions → very technical track.
    """
    n       = rg.randint(12, 20)
    R_base  = rg.uniform(22, 65)
    n_hp    = rg.randint(2, 4)   # number of hairpins
    rot     = rg.uniform(0, 2*math.pi)

    ang = np.linspace(0, 2*math.pi, n, endpoint=False)
    r   = np.full(n, R_base, dtype=float)
    asp = rg.uniform(0.45, 0.90)

    for j in range(n_hp):
        hp_a = rg.uniform(0, 2*math.pi)
        R_min = rg.uniform(0.10, 0.30)*R_base
        w     = rg.uniform(0.08, 0.22)
        delta = np.abs(np.arctan2(np.sin(ang-hp_a), np.cos(ang-hp_a)))
        t = np.clip(1-delta/w, 0, 1)**2
        r = r*(1-(1-R_min/R_base)*t)

    a=R_base; b=R_base*asp
    r_ell = a*b/np.sqrt((b*np.cos(ang))**2+(a*np.sin(ang))**2)
    r = r * r_ell / R_base
    r *= nr.uniform(0.90, 1.10, n)
    ang += nr.uniform(-0.05, 0.05, n)
    return _polar_to_xy(r, ang, rot)

def _arc_samples(p, h, theta, R, step=0.5):
    """Sample an arc (signed angle theta, radius R) starting at p with heading h. Returns pts, end, heading."""
    m=max(2,int(math.ceil(abs(theta)*R/step)))
    t=np.linspace(0,theta,m+1)[1:]
    sg=1.0 if theta>=0 else -1.0
    lx=R*np.sin(np.abs(t)); ly=sg*R*(1-np.cos(t))
    ch,sh=math.cos(h),math.sin(h)
    pts=np.column_stack([p[0]+lx*ch-ly*sh, p[1]+lx*sh+ly*ch])
    return pts, pts[-1].copy(), h+theta

def _shape_technical(rg, nr, hw, target_len=None):
    """
    Technical track built like a real FS layout: a chain of straights and arcs
    (hairpins >150 deg, tight corners, sweepers and slaloms) turning both ways.
    Not star-shaped, so sections can fold back next to each other. The loop is
    closed by adjusting straight lengths (least squares) and rejected if two
    sections get closer than track width + 3 m.
    """
    tgt=target_len if target_len else rg.uniform(260,420)
    r_hp=max(hw+2.0, 4.5-hw+0.3)          # inner border >= 2 m, outer diameter >= 9 m
    d_req=2*hw+3.0                         # >= 3 m between borders of different sections
    rad=math.radians
    for _ in range(400):
        # Features: lists of elements; arcs are ['A', theta, R, can_stretch]
        feats=[]
        n_feat=max(5,int(round(tgt/rg.uniform(32,45))))
        for _f in range(n_feat):
            kind=rg.choices(['hairpin','tight','sweeper','slalom'],weights=[4,4,3,2])[0]
            if kind=='hairpin':
                ft=[['A',rad(rg.uniform(150,200)),rg.uniform(r_hp,r_hp+3),False]]
            elif kind=='tight':
                ft=[['A',rad(rg.uniform(60,120)),rg.uniform(max(r_hp,6),12),True]]
            elif kind=='sweeper':
                ft=[['A',rad(rg.uniform(30,90)),rg.uniform(12,25),True]]
            else:
                ft=[]
                for j in range(rg.randint(3,5)):
                    if j: ft.append(['S',rg.uniform(1,4),False,0.0])
                    ft.append(['A',(-1)**j*rad(rg.uniform(35,60)),rg.uniform(7,12),False])
            if rg.random()<0.5:
                for e in ft:
                    if e[0]=='A': e[1]=-e[1]
            feats.append(ft)

        # Total turning must be +2pi (CCW in generation frame): flip whole features
        # towards it, then spread the small residual over stretchable corners
        def total(): return sum(e[1] for ft in feats for e in ft if e[0]=='A')
        for _k in range(4*n_feat):
            D=2*math.pi-total()
            if abs(D)<rad(40): break
            cands=[ft for ft in feats if abs(D+2*sum(e[1] for e in ft if e[0]=='A'))<abs(D)]
            if not cands: break
            for e in rg.choice(cands):
                if e[0]=='A': e[1]=-e[1]
        D=2*math.pi-total()
        stretch=[e for ft in feats for e in ft if e[0]=='A' and e[3]]
        tot=sum(abs(e[1]) for e in stretch)
        if tot<1e-6 or abs(D)>0.6*tot: continue
        for e in stretch: e[1]+=D*abs(e[1])/tot
        if any(not (rad(15)<=abs(e[1])<=rad(150)) for e in stretch): continue

        el=[['S',rg.uniform(8,20),True,10.0]]   # start straight (split at both ends of the loop)
        for ft in feats:
            el.extend(ft)
            el.append(['S',rg.uniform(4,25),True,3.0])
        el.append(['S',rg.uniform(8,20),True,10.0])

        # Close the loop: adjust adjustable straights with min-norm least squares
        def walk(lengths):
            p=np.zeros(2); h=0.0; dirs=[]; k=0
            for e in el:
                if e[0]=='S':
                    if e[2]: dirs.append((math.cos(h),math.sin(h))); L=lengths[k]; k+=1
                    else: L=e[1]
                    p=p+L*np.array([math.cos(h),math.sin(h)])
                else:
                    _,p,h=_arc_samples(p,h,e[1],e[2])
            return p,np.array(dirs)
        L0=np.array([e[1] for e in el if e[0]=='S' and e[2]])
        mins=np.array([e[3] for e in el if e[0]=='S' and e[2]])
        err,U=walk(L0); U=U.T
        M=U@U.T
        if abs(np.linalg.det(M))<1e-6: continue
        L=L0-U.T@np.linalg.solve(M,err)
        if np.any(L<mins) or np.any(L>80): continue

        # Sample the closed centerline
        pts=[np.zeros((1,2))]; p=np.zeros(2); h=0.0; k=0
        for e in el:
            if e[0]=='S':
                Ls=L[k] if e[2] else e[1]
                if e[2]: k+=1
                m=max(1,int(math.ceil(Ls/0.5)))
                t=np.linspace(0,Ls,m+1)[1:]
                pts.append(p+np.outer(t,[math.cos(h),math.sin(h)])); p=pts[-1][-1].copy()
            else:
                a,p,h=_arc_samples(p,h,e[1],e[2]); pts.append(a)
        P=np.vstack(pts)
        if np.linalg.norm(P[-1]-P[0])>0.5: continue
        P=P[:-1]
        length=_arc_length(P)
        if not (RULE_LENGTH_MIN+10<=length<=RULE_LENGTH_MAX-10): continue
        if _min_clearance(P,max(3*d_req,20.0))<d_req: continue
        P=P[::2]                               # ~1 m control points, spline follows them closely
        return P[:,0],P[:,1]
    raise RuntimeError("technical: no valid layout")

_SHAPES = [
    ('technical',   _shape_technical,    50),
    ('oval',        _shape_oval,         2),
    ('hairpin',     _shape_hairpin,      8),
    ('chicane',     _shape_chicane,      10),
    ('lshape',      _shape_lshape,       8),
    ('teardrop',    _shape_teardrop,     3),
    ('sshape',      _shape_sshape,       3),
    ('random_walk', _shape_random_walk,  12),
    ('irregular',   _shape_irregular,    14),
]
_SHAPE_NAMES   = [s[0] for s in _SHAPES]
_SHAPE_FUNCS   = [s[1] for s in _SHAPES]
_SHAPE_WEIGHTS = [s[2] for s in _SHAPES]
_SHAPE_TOTAL   = sum(_SHAPE_WEIGHTS)

def _pick_shape(rg):
    r=rg.uniform(0,_SHAPE_TOTAL); acc=0
    for i,w in enumerate(_SHAPE_WEIGHTS):
        acc+=w
        if r<acc: return _SHAPE_NAMES[i],_SHAPE_FUNCS[i]
    return _SHAPE_NAMES[-1],_SHAPE_FUNCS[-1]

def _align_to_origin(circuit):
    cl=circuit['center']; start=cl[0].copy()
    idx=min(60,len(cl)-1)
    dx=cl[idx,0]-cl[0,0]; dy=cl[idx,1]-cl[0,1]
    if abs(dx)<1e-8 and abs(dy)<1e-8: dx,dy=1.0,0.0
    ang=math.atan2(dy,dx)+math.pi
    ca,sa=math.cos(-ang),math.sin(-ang)
    def T(pts):
        t=pts-start
        return np.column_stack([t[:,0]*ca-t[:,1]*sa, t[:,0]*sa+t[:,1]*ca])
    for k in ('center','outer','inner','blue','yellow'):
        circuit[k]=T(circuit[k])
    return circuit

# Generation
def generate_circuit(seed, target_length=None, track_width=None,
                     n_ctrl_pts=None, cone_spacing=None,
                     max_attempts=200, align_origin=True, force_shape=None):

    rg0=random.Random(seed)
    if force_shape and force_shape in _SHAPE_NAMES:
        fixed_name=force_shape; fixed_fn=_SHAPE_FUNCS[_SHAPE_NAMES.index(force_shape)]
    else:
        fixed_name,fixed_fn=_pick_shape(rg0)

    best = None
    for attempt in range(max_attempts):
        s  = (seed * 1031 + attempt * 37 + 7) & 0xFFFFFFFF
        rg = random.Random(s)
        nr = np.random.RandomState(s)

        half_w  = (track_width/2.0) if track_width else rg.uniform(1.6, 2.6)
        tgt_len = target_length if target_length else rg.uniform(220, 480)
        spacing = cone_spacing  if cone_spacing  else rg.uniform(2.8, 4.7)

        try:
            if fixed_name=='technical': cx,cy=fixed_fn(rg,nr,half_w,target_length)
            else:                       cx,cy=fixed_fn(rg,nr,half_w)
        except Exception:
            continue
        if len(cx)<4: continue

        smooth=rg.uniform(0.2,1.5)
        try:
            tck,_=splprep([cx,cy],s=smooth,per=1,k=3)
        except Exception:
            try:    tck,_=splprep([cx,cy],s=3.0,per=1,k=3)
            except: continue

        u = np.linspace(0, 1, 5000, endpoint=False)
        xs, ys = splev(u, tck)
        center = np.column_stack([xs, ys])
        raw_l  = _arc_length(center)
        if raw_l < 1.0: continue
        # technical layouts are built in metres; rescale only if a length was requested
        if fixed_name!='technical' or target_length: center *= tgt_len / raw_l

        norms  = _outward_normals(center)
        outer  = _offset(center, norms,  half_w)
        inner  = _offset(center, norms, -half_w)
        blue   = _place_cones_adaptive(outer, spacing)
        yellow = _place_cones_adaptive(inner, spacing)
        if len(blue)<4 or len(yellow)<4: continue

        tl  = _arc_length(center)
        mw  = _min_width(blue, yellow)
        mrd = 2.0 * (_center_min_r(center) + half_w)   # outer border of the tightest turn, either side
        cg  = max(_max_gap(blue), _max_gap(yellow))
        clr_req = 2*half_w + 3.0
        clr = _min_clearance(center, max(3*clr_req, 20.0))

        rules = {
            'length':   {'label':'Track length',    'req':'200 – 500 m','value':tl,  'fmt':f'{tl:.1f} m',  'pass':RULE_LENGTH_MIN<=tl<=RULE_LENGTH_MAX},
            'width':    {'label':'Min track width', 'req':'>= 3 m',     'value':mw,  'fmt':f'{mw:.2f} m',  'pass':mw>=RULE_WIDTH_MIN},
            'outer_d':  {'label':'Min outer diam.', 'req':'>= 9 m',     'value':mrd, 'fmt':f'{mrd:.2f} m', 'pass':mrd>=RULE_OUTER_D_MIN},
            'cone_gap': {'label':'Max cone gap',    'req':'<= 5 m',     'value':cg,  'fmt':f'{cg:.2f} m',  'pass':cg<=RULE_CONE_GAP_MAX},
            'clearance':{'label':'Section clearance','req':f'>= {clr_req:.1f} m','value':clr,'fmt':f'{clr:.1f} m','pass':clr>=clr_req},
        }
        ok = all(r['pass'] for r in rules.values())
        res = {
            'seed':seed,'attempt':attempt,'compliant':ok,'shape':fixed_name,
            'center':center,'outer':outer,'inner':inner,'blue':blue,'yellow':yellow,
            'rules':rules,
            'params':{'length':tl,'width':2*half_w,'spacing':spacing,
                      'n_ctrl':len(cx),'n_blue':len(blue),'n_yellow':len(yellow)},
        }
        if ok:
            return _align_to_origin(res) if align_origin else res
        if best is None: best = res

    r=best if best is not None else res
    return _align_to_origin(r) if align_origin else r

def generate_batch(count,base_seed=2026,**kw):
    return [generate_circuit(_next_seed(base_seed,i),**kw) for i in range(count)]

def _speed_profile(center):
    if _HAS_PROFILES:
        return _arus_profiles(center[:,0].tolist(),center[:,1].tolist())
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
    return {'x':center[:,0].tolist(),'y':center[:,1].tolist(),'s':s,'k':k,
            'speed_profile':sp,'acc_profile':ac}

def export_pcd(circuit,path):
    b=circuit['blue']; y=circuit['yellow']
    bo=np.array([circuit['outer'][0],circuit['inner'][0]])
    all_c=([(p[0],p[1],0) for p in b]+[(p[0],p[1],1) for p in y]+[(p[0],p[1],3) for p in bo])
    hdr=(f"# .PCD v0.7 - Point Cloud Data file format\n"
         f"# FSG 2026  seed={circuit['seed']}  shape={circuit.get('shape','?')}\n"
         f"# start=(0,0) — ARUSSim ready\n"
         f"VERSION 0.7\nFIELDS x y z color score\nSIZE 4 4 4 4 4\nTYPE F F F I F\n"
         f"COUNT 1 1 1 1 1\nWIDTH {len(all_c)}\nHEIGHT 1\n"
         f"VIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(all_c)}\nDATA ascii\n")
    rows="\n".join(f"{x:.6f} {y:.6f} 0.000000 {t} 1" for x,y,t in all_c)
    Path(path).write_text(hdr+rows+"\n")

def export_json(circuit,path):
    Path(path).write_text(json.dumps(_speed_profile(circuit['center']),indent=2))

def export_both(circuit,base):
    export_pcd(circuit,base+'.pcd'); export_json(circuit,base+'.json')

if __name__=='__main__':
    import argparse
    ap=argparse.ArgumentParser()
    ap.add_argument('-n','--count',  type=int,   default=5)
    ap.add_argument('-s','--seed',   type=int,   default=2026)
    ap.add_argument('-o','--out',    type=str,   default='.')
    ap.add_argument('-l','--length', type=float, default=None)
    ap.add_argument('-w','--width',  type=float, default=None)
    ap.add_argument('--spacing',     type=float, default=None)
    ap.add_argument('--shape',       type=str,  default=None,choices=_SHAPE_NAMES)
    ap.add_argument('--all',         action='store_true')
    args=ap.parse_args()
    out=Path(args.out); out.mkdir(parents=True,exist_ok=True)
    new_session(); ok=0
    print(f"\n[ARUS] {args.count} circuits  seed={args.seed}\n")
    for i in range(args.count):
        seed=_next_seed(args.seed,i)
        c=generate_circuit(seed,args.length,args.width,None,args.spacing,force_shape=args.shape)
        st='✓' if c['compliant'] else '✗'
        print(f"  [{st}] {seed:010d}  shape={c['shape']:12s}  L={c['params']['length']:.0f}m  "
              f"cones={c['params']['n_blue']+c['params']['n_yellow']}  att={c['attempt']}")
        if c['compliant']: ok+=1
        if c['compliant'] or args.all: export_both(c,str(out/f"circuit_{seed:010d}"))
    print(f"\n[ARUS] {ok}/{args.count} compliant → {out.resolve()}\n")
