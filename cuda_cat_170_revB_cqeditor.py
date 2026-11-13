"""CUDA-CAT 170 (CQ-editor version): paste into CQ-editor and press F5 to see the car.
This version only builds and shows the model. The full version in the zip also exports STEP/STL,
the 3D viewer page and the fit check.

Hellcat-powered 1969 Barracuda fastback restomod concept -- REV B. Units mm, X forward, Y left, Z up.
Run with CadQuery 2.8; outputs named STEP assembly, STL, mesh data, 3D viewer and a fit report.
Intended real hardware: Hellcat / Demon family supercharged 6.2 Hemi and 8-speed automatic.
Engine/transmission geometry is a simplified envelope built from the DT numbers below,
NOT OEM CAD or verified fit. Measure the donor and type the real numbers into DT.
Ride: custom box frame on air springs. Laid out = suspension sitting on the frame-mounted rubber
bump stops, set where the tires still clear the fenders. Lifted = 10 in above laid out.
The STEP is exported at the as-drawn height, which sits just above laid out.
All chassis, suspension and wheel parameters are design proposals, not fabrication data.
"""
import sys, pathlib, math, json, struct, gzip, base64
try:
    ROOT = pathlib.Path(__file__).resolve().parent
except NameError:          # pasted into CQ-editor: no file path
    ROOT = pathlib.Path.cwd()
import cadquery as cq
from OCP.BRepOffsetAPI import BRepOffsetAPI_ThruSections

P = dict(wheelbase=2743, front_track=1580, rear_track=1570,
         front_tire_diameter=718, rear_tire_diameter=790,
         front_tire_width=335, rear_tire_width=405,
         front_rim_diameter=20*25.4, rear_rim_diameter=22*25.4,
         front_rim_width=12.5*25.4, rear_rim_width=15*25.4,
         front_offset=-12, rear_offset=-32, ride_lift=0,
         rear_lip_raise=4*25.4,
         body_length=4880, body_width=1950)
# ---- Drivetrain envelope (Hellcat / Demon family). Change these to match your measured donor. ----
DT = dict(block_front_x=1630,      # X of the block's front face (timing cover mounts here)
          crank_z_front=425,       # crank centerline height at the block front, above ground
          tilt_deg=2.0,            # engine/trans tilt, rear down (sets driveline angle)
          block_length=560, bank_angle=45, bore_spacing=113.3, bank_stagger=22,
          deck_height=230, skirt_half_width=215,
          head_height=130, head_width=250, cover_height=45, cover_width=190,
          sump_length=230, sump_depth=80, pan_front_depth=15,   # rear-sump pan clears the front K-member
          blower_height=480,       # top of blower lid above the crank centerline
          blower_pulley_z=420, blower_pulley_r=55, crank_pulley_r=105,
          throttle_body_z=315, throttle_body_r=52,
          trans=dict(bell_length=230, bell_r_front=205, bell_r_rear=175,
                     case_length=470, case_r_front=165, case_r_rear=135,
                     tail_length=180, tail_r=75),
          collector_x=1010, collector_y=335, collector_z=285, exhaust_z=234,
          airbox_center=(1790,-430,640), airbox_size=(110,220,200),
          prop_r=45, pinion_length=270,
          front_crossmember_z=255, rear_arch_z=650,
          firewall_x=940, floor_z=340, tunnel_clearance=30,
          scoop_pocket_width_frac=.70, scoop_wall=16, hood_pocket_floor_z=740,
          min_clearance=10, report_gap_under=25)
# ---- Air ride: heights are rocker height above the ground, in inches. ----
RIDE = dict(laid_fender_clearance_mm=10,  # tire-to-fender gap left when sitting on the bump stops
            drive_above_laid_in=2.0,     # normal driving height, above laid out
            lift_above_laid_in=10.0,     # full lift, above laid out
            bag_compressed_mm=75)        # bag stack height when laid out
ARCH_GAP = dict(front=37, rear=25)
# ---- Molded widebody flares (mm). out = how far the flare stands off the body side. ----
FLARE = dict(front=dict(out=40, fade_front=260, fade_rear=200, flat_frac=0.55, full_height_frac=0.55, lip=6),
             rear=dict(out=55, fade_front=300, fade_rear=380, flat_frac=0.55, full_height_frac=0.55, lip=6))
# ---- Suspension geometry (mm). Front: unequal-length A-arms. Rear: Ford 9-inch on a 4-link. ----
SUSP = dict(f_lower_pivot_y=320, f_lower_pivot_z=240, f_lower_bj_z=215,     # lower arm: frame pivot -> ball joint (y=745)
            f_upper_pivot_y=430, f_upper_pivot_z=525, f_upper_bj_z=430, f_upper_spread=240,     # upper arm: frame pivot -> ball joint (y=735)
            r_lower_frame_x=-900, r_lower_frame_z=285, r_lower_y=430,       # 4-link lower bars (axle end below the tube)
            r_upper_frame_x=-900, r_upper_frame_z=520, r_upper_y=200,      # 4-link upper bars (axle end above the tube)
            nine_tube_od=76.2,          # Ford 9-inch: 3 in OD axle tubes
            nine_pinion_drop=57)        # pinion sits about 2-1/4 in below the axle centerline (hypoid offset)       # wheel-opening radius minus tire radius, as drawn
LAID_DZ = -(min(ARCH_GAP.values())-RIDE['laid_fender_clearance_mm'])   # body move from as-drawn to laid out
# ---- Box frame sizes (mm): 2x4 main rails, 2x3 crossmembers, 2x2 X-brace. ----
FRAME = dict(rail=(50.8,101.6), cross=(50.8,76.2), xbrace=(50.8,50.8),
             rear_kickup_z=588)     # C-notch: rails arch over the rear axle so it can lay out
# ---- Front air dam + splitter (owner's L sketch). ----
AERO = dict(splitter_ground_clearance=125,  # owner's sketch: about 5 in at the as-drawn height
            splitter_thickness=12,
            splitter_lip=100,               # how far the blade sticks out past the dam face, about 4 in
            splitter_run_back=270,          # how far the blade runs back under the nose from the dam face
            dam_face_x=2462, dam_top_z=238)
AXLES = [('front', 1343), ('rear', -1400)]
PARTS = {}
COL = dict(body=(.115,.13,.145), black=(.035,.04,.045), bronze=(.52,.32,.14), seam=(.042,.05,.058), cage=(.08,.095,.105), fog=(1.0,.67,.055),
           silver=(.57,.62,.65), tire=(.021,.024,.028), glass=(.075,.145,.19),
           red=(.65,.025,.015), lamp=(.87,.94,1.0), frame=(.14,.16,.18))
def add(name, shapes, color, category='mechanical'):
    if not isinstance(shapes, list): shapes=[shapes]
    PARTS[name]=(shapes,COL.get(color,color),category)
def box(x,y,z,c): return cq.Workplane('XY').box(x,y,z).val().translate(c)
def cyl(r,h,p,d=(0,0,1)): return cq.Solid.makeCylinder(r,h,cq.Vector(*p),cq.Vector(*d))
def tube(a,b,r):
    v=cq.Vector(*b)-cq.Vector(*a)
    return cq.Solid.makeCylinder(r,v.Length,cq.Vector(*a),v.normalized())
def boxtube(a,b,size):
    """Rectangular box tube from a to b; size=(width, height), height kept as vertical as possible."""
    A=cq.Vector(*a);d=cq.Vector(*b)-A;n=d.normalized()
    side=n.cross(cq.Vector(0,0,1))
    if side.Length<1e-6:side=cq.Vector(0,1,0)
    side=side.normalized()
    w,h=size
    return cq.Workplane(cq.Plane(origin=A,xDir=side,normal=n)).rect(w,h).extrude(d.Length).val()
def ring(ro,ri,w,p): return cyl(ro,w,p,(0,1,0)).cut(cyl(ri,w,p,(0,1,0)))
def wire(points): return cq.Wire.makePolygon([cq.Vector(*p) for p in points],close=True)
def loft(sections): return cq.Solid.makeLoft([wire(s) for s in sections],False)
def fair_body_loft(sections):
    """Cubic skin prevents a long multi-station loft from developing wavy shoulders."""
    builder=BRepOffsetAPI_ThruSections(True,False,1e-6)
    builder.SetMaxDegree(3)
    for section in sections:builder.AddWire(wire(section).wrapped)
    builder.Build()
    return cq.Solid(builder.Shape())
def panel(points,t=4):
    return cq.Compound.makeCompound([cq.Solid.extrudeLinear(wire([points[0],points[i],points[i+1]]),[],cq.Vector(0,0,t)) for i in range(1,len(points)-1)])
def surface_line(shape,path,side=None,radius=2.3):
    """Place a fine panel reveal on the actual lofted skin, splitting at openings."""
    result=[]; previous=None
    for a,b in zip(path,path[1:]):
        n=max(1,math.ceil(math.dist(a,b)/35))
        for k in range(n+1):
            u=k/n;p=[a[j]*(1-u)+b[j]*u for j in range(2)]
            if side is None:
                ray=cq.Edge.makeLine(cq.Vector(p[0],p[1],0),cq.Vector(p[0],p[1],1450))
            else:
                ray=cq.Edge.makeLine(cq.Vector(p[0],0,p[1]),cq.Vector(p[0],side*1200,p[1]))
            vertices=[v.toTuple() for v in shape.intersect(ray).Vertices()]
            if not vertices:previous=None;continue
            q=max(vertices,key=lambda v: v[2] if side is None else abs(v[1]))
            q=(q[0],q[1],q[2]+.5) if side is None else (q[0],q[1]+side*.5,q[2])
            if previous and 0.1<math.dist(previous,q)<70:result.append(tube(previous,q,radius))
            previous=q
    return result
def body_section(x,w,z,low=190):
    # Shallow tucked waist below the shoulder: retains the overall envelope.
    return [(x,-w*.87,low),(x,-w,low+95),(x,-w*.984,low+245),(x,-w,z-95),(x,-w*.94,z),
            (x,0,z+14),(x,w*.94,z),(x,w,z-95),(x,w*.984,low+245),(x,w,low+95),(x,w*.87,low)]
def arches(shape):
    for axle,x in AXLES:
        center=P[axle+'_tire_diameter']/2
        radius=center+ARCH_GAP[axle]
        shape=shape.cut(cyl(radius,2600,(x,-1300,center),(0,1,0)))
        shape=shape.cut(box(2*radius,2600,center+300,(x,0,(center-300)/2)))   # straight sides below the axle line
    return shape

def build():
    print('Building body and fastback cabin',flush=True)
    # Lower body shell, using station lofts rather than a rectangular block.
    # Fair the plan-view outline instead of pinching the doors between isolated bulges.
    stations=[(-2440,955,840),(-2240,970,845),(-1800,985,850),(-1400,990,855),
              (-850,980,820),(0,965,815),(700,965,810),(1343,975,815),
              (1850,935,780),(2320,860,705),(2440,825,650)]
    shell=fair_body_loft([body_section(*s) for s in stations])
    # Hollow interior and engine compartment, open bottom for chassis inspection.
    void=box(4380,1480,1200,(0,0,360))
    shell=arches(shell.cut(void))
    add('Body_shell',shell,'body','body')
    # Fastback roof with empty cabin, separately glazed apertures.
    roof=loft([[(x,-w,z-24),(x,w,z-24),(x,w,z),(x,0,z+28),(x,-w,z)]
               for x,w,z in [(-720,650,1265),(-400,674,1290),(150,670,1290),(470,635,1260)]])
    add('Fastback_roof',roof,'body','body')
    pillars=[]
    for s in [-1,1]:
        pillars += [tube((-370,s*865,823),(-400,s*670,1275),14)]
        # Side window plane tapers into the roof; two panes with visible B-pillar.
        add('Side_glass_'+str(s),[
            panel([(965,s*800,835),(455,s*638,1238),(-365,s*666,1254),(-340,s*862,835)]),
            panel([(-398,s*862,835),(-427,s*666,1254),(-706,s*648,1240),(-1260,s*817,835)])],
            'glass','body')
        add('Fastback_C_pillar_'+str(s),panel([(-720,s*650,1265),(-1260,s*817,835),(-1905,s*800,847)]),'body','body')
    add('Pillars',pillars,'body','body')
    add('Windshield',panel([(986,-790,830),(986,790,830),(458,619,1247),(458,-619,1247)]),'glass','body')
    add('Fastback_rear_glass',panel([(-1880,-760,857),(-1880,760,857),(-722,615,1248),(-722,-615,1248)]),'glass','body')
    # Continuous semi-recessed 2-inch OD cage tying glass surrounds to the chassis.
    cage_front=[];cage_roof=[];cage_rear=[];cage_lower=[];cage_braces=[];cage_window_outline=[]
    for s in [-1,1]:
        front=[(720,s*780,315),(1000,s*810,812),(470,s*635,1262)]
        cage_front += [tube(a,b,25.4) for a,b in zip(front,front[1:])]
        roofline=[(470,s*635,1262),(-400,s*670,1290),(-720,s*650,1265)]
        cage_roof += [tube(a,b,25.4) for a,b in zip(roofline,roofline[1:])]
        # Owner's purple outline: wrap the roof edge and descend the quarter-window line.
        window_top=[(470,s*635,1262),(455,s*638,1238),(-427,s*666,1254),(-706,s*648,1240),(-720,s*650,1265)]
        cage_window_outline += [tube(a,b,25.4) for a,b in zip(window_top,window_top[1:])]
        # Quarter-window leg follows the outline, then steps forward over the tire and drops ahead of the rear wheel.
        quarter_leg=[(-706,s*648,1240),(-1260,s*817,835),(-930,s*720,760),(-930,s*720,315)]
        cage_window_outline += [tube(a,b,25.4) for a,b in zip(quarter_leg,quarter_leg[1:])]
        rear=[(-720,s*650,1265),(-1920,s*800,847),(-2050,s*700,310)]
        cage_rear += [tube(a,b,25.4) for a,b in zip(rear,rear[1:])]
        cage_rear += [tube((-400,s*760,315),(-400,s*670,1290),25.4)]
        lower=[(720,s*780,315),(-400,s*760,315),(-930,s*720,315)]   # sill bar stops ahead of the rear wheel
        cage_lower += [tube(a,b,25.4) for a,b in zip(lower,lower[1:])]
        for x,y,z in [(720,780,315),(-400,760,315),(-2050,700,310)]:
            cage_lower.append(tube((x,s*y,z),(x,s*570,285),25.4))
        cage_lower.append(tube((-930,s*720,315),(-950,s*545,300),25.4))   # tie into the frame ahead of the rear wheel
        cage_braces.append(tube((-750,s*555,1204),(-1825,-s*685,824),19.05))
    for x,w,z in [(470,635,1262),(-400,670,1290),(-720,650,1265)]:
        cage_roof.append(tube((x,-w,z),(x,w,z),25.4))
    cage_front.append(tube((1000,-810,812),(1000,810,812),22.2))
    cage_rear.append(tube((-1920,-800,847),(-1920,800,847),22.2))
    add('Cage_windshield_and_A_legs',cage_front,'cage')
    add('Cage_roof_rails_and_headers',cage_roof,'cage')
    add('Cage_main_hoop_and_fastback_legs',cage_rear,'cage')
    add('Cage_sill_rails_and_chassis_ties',cage_lower,'cage')
    add('Cage_rear_window_X_braces',cage_braces,'cage')
    add('Cage_roof_and_quarter_window_outline',cage_window_outline,'cage')
    # Ten tapered louvers follow the fastback backlight, leaving open slots.
    louvers=[];louver_frames=[]
    def backlight(u):return (-722-1158*u,615+145*u,1248-391*u)
    for i in range(10):
        u=.065+i*.095;x,w,z=backlight(u)
        louvers.append(loft([[(xx,-ww,zz),(xx,ww,zz),(xx,ww,zz+7),(xx,-ww,zz+7)]
                            for xx,ww,zz in [(x+37,w-4,z+31),(x-55,w+8,z+48)]]))
    for s in [-1,1]:
        louver_frames += [tube((-730,s*622,1271),(-1872,s*766,881),9)]
        louver_frames += [tube((-735,s*340,1263),(-1865,s*420,881),7)]
    louver_frames += [tube((-730,-622,1271),(-730,622,1271),9),tube((-1872,-766,881),(-1872,766,881),9)]
    add('Fastback_rear_window_louver_blades',louvers,'black','louvers')
    add('Fastback_louver_frame_and_spines',louver_frames,'black','louvers')
    # Hood/deck are separate parts, allowing a useful mechanical cutaway.
    hood=loft([[ (x,-w,z),(x,w,z),(x,w,z+13),(x,0,z+29),(x,-w,z+13)]
               for x,w,z in [(980,766,803),(1750,787,790),(2285,756,720)]])
    add('Hood',hood,'black','hood')
    def hood_top(x,y):
        hit=hood.intersect(cq.Edge.makeLine(cq.Vector(x,y,0),cq.Vector(x,y,1400)))
        return max(v.toTuple()[2] for v in hit.Vertices())
    # Fair transverse profiles roll the scoop's shoulders into the existing hood skin.
    scoop_sections=[]
    scoop_stations=[(985,365,1.0),(1085,385,94),(1250,402,126),(1580,410,132),(1820,365,123),(1940,315,105)]
    for x,w,h in scoop_stations:
        top=[(x,w*f,hood_top(x,w*f)+h*rise) for f,rise in
             [(-1,0),(-.96,.08),(-.83,.56),(-.57,.92),(0,1),(.57,.92),(.83,.56),(.96,.08),(1,0)]]
        scoop_sections.append([(x,-w,hood_top(x,-w)-7)]+top+[(x,w,hood_top(x,w)-7)])
    scoop=fair_body_loft(scoop_sections)
    # A shallow inset mouth reads as an inlet, instead of a rectangular block on the hood.
    scoop_inlet=cq.Solid.extrudeLinear(wire([(1927,-218,821),(1927,218,821),(1927,205,865),(1927,-205,865)]),[],cq.Vector(25,0,0))
    scoop=scoop.cut(scoop_inlet)
    add('Blended_hood_scoop',scoop,'black','hood')
    add('Scoop_recessed_inlet',box(5,425,40,(1925,0,842)),'tire','hood')
    add('Rear_deck',loft([[ (x,-w,z),(x,w,z),(x,w,z+16),(x,-w,z+16)]
                       for x,w,z in [(-2435,808,840),(-2200,825,845),(-1950,800,850)]]),'body','body')
    # Raise the lip's trailing edge four inches; preserve its attached leading root.
    add('Ducktail',loft([[ (x,-w,z),(x,w,z),(x,w,z+15+P['rear_lip_raise']*rise),(x,-w,z+15+P['rear_lip_raise']*rise)]
                       for x,w,z,rise in [(-2445,810,864,1),(-2375,820,843,0)]]),'body','body')
    print('Defining hood, fender, door and deck panel lines',flush=True)
    body_reveals=[];shoulder=[];lower_character=[];hood_reveals=[];hood_stamps=[]
    for s in [-1,1]:
        # Complete door reveals fitted to the actual side surface, replacing floating lines.
        body_reveals += surface_line(shell,[(-690,797),(-713,725),(-700,308),(-660,285),(610,285),(650,310),(730,735),(760,785)],s)
        # Leading and trailing clip joins / rear-quarter separation from tail cap.
        body_reveals += surface_line(shell,[(2288,727),(2310,660),(2310,290)],s)
        body_reveals += surface_line(shell,[(-2300,832),(-2310,740),(-2310,290)],s)
        # A continuous shoulder crease follows the fender bulges and stops at wheel openings.
        shoulder += surface_line(shell,[(x,z-108) for x,w,z in stations[1:-1]],s,1.5)
        lower_character += surface_line(shell,[(-2310,325),(-1800,336),(-700,343),(650,343),(1800,336),(2300,325)],s,1.2)
        # Hood joint and twin longitudinal stamp lines, sampled from its top face.
        hood_reveals += surface_line(hood,[(986,s*752),(1750,s*773),(2278,s*744)],radius=2.8)
        hood_stamps += surface_line(hood,[(1020,s*432),(1710,s*445),(2240,s*438)],radius=1.8)
    hood_reveals += surface_line(hood,[(990,-745),(990,745)],radius=2.8)
    hood_reveals += surface_line(hood,[(2278,-740),(2278,740)],radius=2.8)
    deck=PARTS['Rear_deck'][0][0]
    deck_reveals=surface_line(deck,[(-1970,-785),(-1970,785)],radius=2.4)
    for s in [-1,1]:deck_reveals+=surface_line(deck,[(-1970,s*785),(-2200,s*810),(-2365,s*797)],radius=2.4)
    add('Door_fender_and_tail_panel_reveals',body_reveals,'seam','body')
    add('Fender_and_quarter_shoulder_crease',shoulder,'seam','body')
    add('Lower_body_character_line',lower_character,'cage','body')
    add('Hood_perimeter_panel_gaps',hood_reveals,'body','hood')
    add('Hood_longitudinal_stamp_lines',hood_stamps,'body','hood')
    add('Decklid_panel_gaps',deck_reveals,'seam','body')
    # Molded widebody flares: smooth humps that roll out of the body side over each wheel and fade
    # back into the door seam / tail. Built from the body's own section shape so they blend cleanly.
    flares=[]; lips=[]; accents=[]
    skin_cache={}
    def side_skin(x,z,s):
        cache_key=(round(x,5),round(z,5))
        if cache_key in skin_cache:return s*skin_cache[cache_key]
        ray=cq.Edge.makeLine(cq.Vector(x,0,z),cq.Vector(x,s*1200,z))
        vertices=[v.toTuple() for v in shell.intersect(ray).Vertices()]
        if not vertices:raise ValueError('Skin not found at '+str((x,z)))
        y=max(vertices,key=lambda v:abs(v[1]))[1]
        skin_cache[cache_key]=abs(y)
        return y
    def station_at(x):
        for (x0,w0,z0),(x1,w1,z1) in zip(stations,stations[1:]):
            if x0<=x<=x1:
                t=(x-x0)/(x1-x0);return w0+t*(w1-w0),z0+t*(z1-z0)
        return (stations[0][1],stations[0][2]) if x<stations[0][0] else (stations[-1][1],stations[-1][2])
    def skin_y(x,z,low=190):
        w,zt=station_at(x)
        prof=[(low,.87),(low+95,1.0),(low+245,.984),(zt-95,1.0),(zt,.94)]
        for (za,fa),(zb,fb) in zip(prof,prof[1:]):
            if za<=z<=zb:return w*(fa+(z-za)/(zb-za)*(fb-fa))
        return w*(prof[0][1] if z<low else prof[-1][1])
    def smooth(t):t=min(max(t,0),1);return t*t*(3-2*t)
    for axle,xw in AXLES:
        center=P[axle+'_tire_diameter']/2; radius=center+ARCH_GAP[axle]
        FL=FLARE[axle]
        span_f,span_r=FL['fade_front'],FL['fade_rear']
        xs=[xw-radius-span_r+i*(2*radius+span_r+span_f)/16 for i in range(17)]
        def off(x):
            d=(x-xw)
            if abs(d)<=radius*FL['flat_frac']:return FL['out']
            if d>0:t=(d-radius*FL['flat_frac'])/(radius*(1-FL['flat_frac'])+span_f)
            else:t=(-d-radius*FL['flat_frac'])/(radius*(1-FL['flat_frac'])+span_r)
            return FL['out']*(1-smooth(t))
        def pz(z,zt):
            top_full=center+radius*FL['full_height_frac']
            return 1.0 if z<=top_full else 1-smooth((z-top_full)/(zt-95-top_full))
        for s in [-1,1]:
            sections=[]
            for x in xs:
                w,zt=station_at(x)
                zs=[190+i*(zt-95-190)/14 for i in range(15)]
                outer=[(x,s*(skin_y(x,z)+off(x)*pz(z,zt)),z) for z in zs]
                inner=[(x,s*(skin_y(x,z)-30),z) for z in reversed(zs)]
                sections.append(outer+inner)
            fl=fair_body_loft(sections)
            fl=fl.cut(cyl(radius,2600,(xw,-1300,center),(0,1,0))).cut(box(2*radius,2600,center+300,(xw,0,(center-300)/2)))
            flares.append(fl)
            # Crisp lip around the opening, sitting on the flare's outer face.
            secs=[]
            for i in range(25):
                a_=math.radians(-6+i*(192/24))
                ring_=[]
                for dr,dy in [(0,0),(0,FL['lip']),(14,FL['lip']),(14,0)]:
                    xx=xw+(radius+dr)*math.cos(a_);zz=center+(radius+dr)*math.sin(a_)
                    w,zt=station_at(xx)
                    yy=skin_y(xx,zz)+off(xx)*pz(zz,zt)
                    ring_.append((xx,s*(yy-6+dy),zz))
                secs.append(ring_)
            lips.append(cq.Solid.makeLoft([wire(r_) for r_ in secs],False))
    for s in [-1,1]:
        accents += [box(1770,45,65,(-10,s*908,211))]
        # Flush handles, visible from both sides.
        add('Handle_'+str(s),box(115,14,20,(-515,side_skin(-515,746,s)+s*8,746)),'silver','body')
        add('Mirror_'+str(s),[tube((740,s*820,908),(700,s*990,950),12),
                              box(140,90,65,(700,s*1000,959))],'black','body')
    add('Molded_integrated_fender_flares',flares,'body','body'); add('Fender_flare_lips',lips,'body','body'); add('Rockers_and_panel_lines',accents,'black','body')
    # Reference-style classic face: full-width surround and two horizontal inserts.
    def grille_wire(x,w,h,corner):
        z=525
        return wire([(x,-w+corner,z-h),(x,w-corner,z-h),(x,w,z-h+corner),
                     (x,w,z+h-corner),(x,w-corner,z+h),(x,-w+corner,z+h),
                     (x,-w,z+h-corner),(x,-w,z-h+corner)])
    opening=cq.Solid.extrudeLinear(grille_wire(2220,804,126,35),[],cq.Vector(280,0,0))
    shell=shell.cut(opening)
    for s in [-1,1]:shell=shell.cut(cyl(110,280,(2220,s*660,529),(1,0,0)))
    add('Body_shell',shell,'body','body')
    duct_outer=cq.Solid.makeLoft([grille_wire(2260,520,82,12),grille_wire(2448,551,105,18)],False)
    duct_inner=cq.Solid.makeLoft([grille_wire(2258,510,72,10),grille_wire(2451,536,90,16)],False)
    add('Grille_recessed_tapered_duct',duct_outer.cut(duct_inner),'black','body')
    add('Grille_shadow_backing',box(8,1040,165,(2262,0,525)),'tire','body')
    surround=cq.Solid.extrudeLinear(grille_wire(2458,810,133,38),[grille_wire(2458,796,119,34)],cq.Vector(13,0,0))
    add('Grille_full_width_bright_surround',surround,'silver','body')
    mask=box(30,1590,238,(2438,0,525)).cut(cq.Solid.extrudeLinear(grille_wire(2418,537,91,16),[],cq.Vector(50,0,0)))
    for s in [-1,1]:mask=mask.cut(cyl(102,60,(2413,s*660,529),(1,0,0)))
    add('Grille_black_full_width_face',mask,'black','body')
    mesh=[box(7,2,100,(2404,y,550)) for y in range(-510,511,22) if abs(y)>65]
    add('Grille_fine_vertical_back_mesh',mesh,'cage','body')
    fins=[box(26,1040,3.5,(2429,0,z)) for z in range(502,599,12)]
    add('Grille_horizontal_blades',fins,'cage','body')
    inserts=[]
    for s in [-1,1]:
        inserts.append(cq.Solid.extrudeLinear(grille_wire(2442,242,56,9),[grille_wire(2442,232,47,7)],cq.Vector(8,0,0)).translate((0,s*285,29)))
    add('Twin_grille_insert_frames',inserts,(.30,.34,.36),'body')
    center=loft([[(x,-w,427),(x,w,427),(x,w,623),(x,-w,623)] for x,w in [(2395,75),(2468,55)]])
    add('Grille_center_divider',center,'black','body')
    bumper=loft([[(xf-50,y,386),(xf,y,386),(xf,y,410),(xf-50,y,410)]
                 for y,xf in [(-825,2450),(-600,2500),(0,2515),(600,2500),(825,2450)]])
    add('Slim_bright_front_bumper',bumper,'silver','body')
    lower_cut=box(300,1290,115,(2380,0,315))
    shell=shell.cut(lower_cut)
    for s_ in [-1,1]:  # exhaust exits: notches in the rear valance above the diffuser
        shell=shell.cut(box(360,150,170,(-2320,s_*435,265)))
    add('Body_shell',shell,'body','body')
    add('Lower_air_opening',box(12,1290,111,(2420,0,315)),'tire','body')
    add('Lower_intake_fins',[box(16,930,3,(2444,0,z)) for z in [288,308,328,348]],'cage','body')
    fog_buckets=[];fog_bezels=[];fog_lenses=[]
    for s in [-1,1]:
        fog_buckets.append(cyl(67,52,(2420,s*550,315),(1,0,0)).cut(cyl(54,56,(2418,s*550,315),(1,0,0))))
        fog_bezels.append(cyl(59,9,(2470,s*550,315),(1,0,0)).cut(cyl(53,13,(2468,s*550,315),(1,0,0))))
        fog_lenses.append(cyl(52,9,(2470,s*550,315),(1,0,0)))
    add('Fog_light_recessed_buckets',fog_buckets,'black','body')
    add('Fog_light_bright_bezels',fog_bezels,'silver','body')
    add('Fog_lights_below_bumper',fog_lenses,'fog','body')
    lamps=[]; bezels=[];buckets=[]
    for s in [-1,1]:
        lamps += [cyl(85,15,(2444,s*660,529),(1,0,0))]
        bezels += [cyl(99,13,(2451,s*660,529),(1,0,0)).cut(cyl(87,17,(2449,s*660,529),(1,0,0)))]
        buckets += [cyl(109,77,(2377,s*660,529),(1,0,0)).cut(cyl(100,81,(2375,s*660,529),(1,0,0)))]
    add('Recessed_headlight_buckets',buckets,'black','body')
    add('Headlights',lamps,'lamp','body');add('Headlight_bezels',bezels,'silver','body')
    add('Tail_panel',box(35,1530,155,(-2445,0,650)),'black','body')
    add('Tail_lights',[box(12,360,64,(-2468,s*506,665)) for s in [-1,1]],'red','body')
    add('Rear_bumper',box(70,1710,30,(-2453,0,450)),'silver','body')
    add('Rear_diffuser',[box(180,1420,20,(-2380,0,235))]+[box(210,12,75,(-2380,y,267)) for y in [-550,-275,0,275,550]],'black','body')

    print('Building staggered wheels and brakes',flush=True)
    tires=[]; rims=[]; spokes=[]; rotors=[]; calipers=[]; lugs=[]
    for axle,x in AXLES:
        diameter=P[axle+'_tire_diameter'];R=diameter/2; W=P[axle+'_tire_width']
        rd=P[axle+'_rim_diameter']/2; track=P[axle+'_track'];rw=P[axle+'_rim_width']
        for s in [-1,1]:
            y=s*track/2; z=R
            # Rounded tire cross section revolved about Y then translated to wheel.
            profile=[(rd-7,-W/2+20),(rd+10,-W/2),(R-35,-W/2),(R-8,-W/2+30),
                     (R,-W/2+65),(R,W/2-65),(R-8,W/2-30),(R-35,W/2),
                     (rd+10,W/2),(rd-7,W/2-20)]
            tire=cq.Workplane('XY').polyline(profile).close().revolve(360,(0,0),(0,1)).val().translate((x,y,z))
            # Four circumferential channels cut into the actual tire solid.
            for off in [-W*.28,-W*.095,W*.095,W*.28]:
                tire=tire.cut(ring(R+2,R-5,6,(x,y+off-3,z)))
            tires.append(tire)
            rims += [ring(rd,rd-12,rw,(x,y-rw/2,z))]
            outer=y+s*(rw/2+2); face=outer-s*(52 if axle=='front' else 94)
            rims += [ring(rd+4,rd-8,9,(x,outer-4,z)),ring(rd-9,rd-20,7,(x,face-3,z))]
            spokes += [cyl(53,22,(x,face-11,z),(0,1,0))]
            for k in range(5):
                a=k*2*math.pi/5
                for delta in [-.07,.07]:
                    p=(x+50*math.cos(a+delta),face-s*9,z+50*math.sin(a+delta))
                    q=(x+(rd-22)*math.cos(a+delta+.12),face+s*20,z+(rd-22)*math.sin(a+delta+.12))
                    spokes.append(tube(p,q,11))
                lugs += [cyl(7,12,(x+34*math.cos(a),face+s*18,z+34*math.sin(a)),(0,s,0))]
            by=y+s*P[axle+'_offset']
            rotor=ring(190 if axle=='front' else 180,72,11,(x,by,z))
            rotors.append(rotor)
            calipers.append(box(78,55,140,(x-157,by,z+45)))
    add('Tires',tires,'tire','wheels');add('Wheel_barrels_and_lips',rims,'bronze','wheels')
    add('Forged_split_spokes',spokes,'bronze','wheels');add('Wheel_lugs',lugs,'silver','wheels')
    add('Brake_rotors',rotors,'silver','wheels');add('Brake_calipers',calipers,'red','wheels')

    print('Building front air dam and splitter',flush=True)
    # Owner's sketch: dam drops straight down under the bumper, flat blade pokes out front and runs back under the nose.
    A=AERO; blade_z=A['splitter_ground_clearance']
    def plan(front_x,sweep,half,back_x,n=12):
        front=[(front_x-sweep*(y/half)**2,y) for y in [half*(-1+2*i/n) for i in range(n+1)]]
        return front+[(back_x,half),(back_x,-half)]
    dam_face=A['dam_face_x']
    dam_front=plan(dam_face,95,800,0)[:-2]
    dam_outline=dam_front+[(x-24,y) for x,y in reversed(dam_front)]
    dam=cq.Solid.extrudeLinear(wire([(x,y,blade_z+A['splitter_thickness']) for x,y in dam_outline]),[],
                               cq.Vector(0,0,A['dam_top_z']-blade_z-A['splitter_thickness']))
    add('Front_air_dam',dam,'black','body')
    blade=cq.Solid.extrudeLinear(wire([(x,y,blade_z) for x,y in plan(dam_face+A['splitter_lip'],120,820,dam_face-A['splitter_run_back'])]),[],
                                 cq.Vector(0,0,A['splitter_thickness']))
    add('Front_splitter',blade,'black','body')
    add('Splitter_support_rods',[tube((dam_face-A['splitter_run_back']+40,s*430,blade_z+A['splitter_thickness']),(2150,s*500,300),9) for s in [-1,1]],'silver','body')

    print('Building custom frame and air suspension',flush=True)
    rails=[];links=[];bellows=[]; shocks=[];lower_plates=[];upper_plates=[]
    for s in [-1,1]:
        # Rear rails pinch in to y=500 so they clear the 405-wide rear tires, and C-notch up over the axle.
        pts=[(-2230,s*570,285),(-1850,s*520,320),(-1590,s*500,FRAME['rear_kickup_z']),(-1210,s*500,FRAME['rear_kickup_z']),
             (-950,s*520,285),(-700,s*570,285),(600,s*570,285),(1000,s*480,320),(1750,s*480,320),(2190,s*550,320)]
        rails += [boxtube(a,b,FRAME['rail']) for a,b in zip(pts,pts[1:])]
    # Rev B: crossmembers re-placed around the Hellcat-family drivetrain.
    #  - x=150 and x=750 straight tubes removed (they ran through the 8-speed).
    #  - front K-member dropped under the oil pan, rear crossmember arched over the axle center section.
    for x,z,w in [(-2200,285,570),(2100,320,530)]:
        rails += [boxtube((x,-w,z),(x,w,z),FRAME['cross'])]
    kz=DT['front_crossmember_z']
    k=[(1343,-480,320),(1343,-300,kz),(1343,300,kz),(1343,480,320)]
    rails += [boxtube(a,b,FRAME['cross']) for a,b in zip(k,k[1:])]
    rear_arch=[(-1400,-500,FRAME['rear_kickup_z']),(-1400,-240,DT['rear_arch_z']),(-1400,240,DT['rear_arch_z']),(-1400,500,FRAME['rear_kickup_z'])]
    rails += [boxtube(a,b,FRAME['cross']) for a,b in zip(rear_arch,rear_arch[1:])]
    # Rear crossmember with a hoop OVER the driveshaft (an under-hoop would hit it at full lift).
    hoop=[(-850,-545,285),(-850,-210,285),(-850,-130,470),(-850,130,470),(-850,210,285),(-850,545,285)]
    rails += [boxtube(a,b,FRAME['cross']) for a,b in zip(hoop,hoop[1:])]
    # Tunnel side rails replace the X-brace (an X under the driveshaft would hit it at full lift).
    for s_ in [-1,1]:rails.append(boxtube((-850,s_*235,285),(560,s_*235,285),FRAME['xbrace']))
    add('Custom_box_frame',rails,'frame')
    S_=SUSP; rz=P['rear_tire_diameter']/2; tube_r=S_['nine_tube_od']/2
    fl=[];fu=[];rl=[];ru=[];sub=[];towers=[];r_brk=[];f_brk=[];bellows=[];f_lo=[];r_lo=[];up=[];up_brk=[];stops=[];stop_brk=[]
    def interp(y,y0,z0,y1,z1):return z0+(y-y0)/(y1-y0)*(z1-z0)
    for s in [-1,1]:
        x=1343
        # Front lower A-arm: pivots on a sub-rail welded to the K-member.
        for dx in [-170,170]:
            fl.append(tube((x+dx,s*S_['f_lower_pivot_y'],S_['f_lower_pivot_z']),(x,s*745,S_['f_lower_bj_z']),17))
            ux=x+(S_['f_upper_spread'] if dx>0 else -S_['f_upper_spread'])
            fu.append(tube((ux,s*S_['f_upper_pivot_y'],S_['f_upper_pivot_z']),(x,s*735,S_['f_upper_bj_z']),15))
            towers.append(tube((ux,s*480,371),(ux,s*(S_['f_upper_pivot_y']+10),S_['f_upper_pivot_z']+10),20))
            # rubber bump stop under the frame rail, over the lower arm: touches when laid out
            fy=480; frac=(fy-S_['f_lower_pivot_y'])/(745-S_['f_lower_pivot_y']); w_=(745-fy)/(745-S_['f_lower_pivot_y'])
            ax=x+dx*(1-frac); atop=interp(fy,S_['f_lower_pivot_y'],S_['f_lower_pivot_z'],745,S_['f_lower_bj_z'])+17
            sb=atop-(1-w_)*LAID_DZ; rail_bot=320-FRAME['rail'][1]/2
            stops.append(cyl(22,rail_bot-sb,(ax,s*fy,sb)))
        sub.append(boxtube((x-215,s*S_['f_lower_pivot_y'],S_['f_lower_pivot_z']),(x+215,s*S_['f_lower_pivot_y'],S_['f_lower_pivot_z']),(50.8,50.8)))
        for dx in [-190,190]:sub.append(tube((x+dx,s*S_['f_lower_pivot_y'],S_['f_lower_pivot_z']),(x+dx,s*480,300),18))
        # Front air strut (bag over shock): shock eye on the lower arm, bellows above the upper arm,
        # top cap on a frame tower that straddles the bellows.
        y_b=545; lo=interp(y_b,S_['f_lower_pivot_y'],S_['f_lower_pivot_z'],745,S_['f_lower_bj_z'])+17+6
        w_lo=(745-y_b)/(745-S_['f_lower_pivot_y']); hi=740
        if s==1:BAGS.append((lo,hi,w_lo))
        bel0=585
        bellows.append(cyl(25,bel0-lo,(x,s*y_b,lo)))                       # shock body
        for i in range(4):bellows.append(cyl(65 if i%2 else 58,(hi-12-bel0)/4,(x,s*y_b,bel0+i*(hi-12-bel0)/4)))
        f_lo.append(box(40,40,24,(x,s*y_b,lo-6)))                         # lower eye on the arm
        up.append(cyl(50,12,(x,s*y_b,hi-12)))
        for dx2 in [-95,95]:up_brk.append(tube((x+dx2,s*480,371),(x+dx2,s*(y_b-25),hi),16))
        up_brk.append(box(230,90,14,(x,s*(y_b-20),hi+7)))
        # ---- Rear: Ford 9-inch on a 4-link (bars run front-to-back, the way a solid axle wants them).
        xr=-1400
        rl.append(tube((S_['r_lower_frame_x'],s*S_['r_lower_y'],S_['r_lower_frame_z']),(xr,s*S_['r_lower_y'],rz-65),19))
        ru.append(tube((S_['r_upper_frame_x'],s*S_['r_upper_y'],S_['r_upper_frame_z']),(xr,s*S_['r_upper_y'],rz+85),17))
        r_brk += [box(80,40,50,(xr,s*S_['r_lower_y'],rz-55)),box(80,40,70,(xr,s*S_['r_upper_y'],rz+68))]
        f_brk += [box(50,40,60,(S_['r_lower_frame_x']+15,s*S_['r_lower_y'],S_['r_lower_frame_z'])),
                  box(60,50,S_['r_upper_frame_z']-285+20,(S_['r_upper_frame_x']+15,s*S_['r_upper_y'],(S_['r_upper_frame_z']+285)/2))]
        # Rear air strut sits on top of the 9-inch tube, under the C-notched rail.
        # Rear air spring sits on the 9-inch tube, right under the C-notched rail.
        top=FRAME['rear_kickup_z']-FRAME['rail'][1]/2
        lo=rz+tube_r+12; hi=top-12
        if s==1:BAGS.append((lo,hi,0.0))
        for i in range(5):bellows.append(cyl(68 if i%2 else 62,(hi-lo)/5,(xr,s*500,lo+i*(hi-lo)/5)))
        r_lo.append(cyl(72,12,(xr,s*500,lo-12)));up.append(cyl(72,12,(xr,s*500,hi)))
        # Rear rubber bump stops hang from the arched crossmember and land on top of the 9-inch center.
        banjo_top=rz+165; cm_bot=DT['rear_arch_z']-FRAME['cross'][1]/2
        stops.append(cyl(30,cm_bot-(banjo_top-LAID_DZ),(xr,s*60,banjo_top-LAID_DZ)))
    add('Front_lower_arms',fl,'silver');add('Front_upper_arms',fu,'silver')
    add('Front_subframe_and_arm_towers',sub+towers+up_brk,'frame')
    add('Rear_4link_lower_bars',rl,'silver');add('Rear_4link_upper_bars',ru,'silver')
    add('Rear_4link_frame_brackets',f_brk,'frame');add('Rear_4link_axle_brackets',r_brk,'frame')
    add('Air_springs',bellows,'tire');add('Front_bag_lower_plates',f_lo,'silver');add('Rear_bag_lower_plates',r_lo,'silver')
    add('Air_spring_upper_plates',up,'silver');add('Bump_stops',stops,'tire')
    add('Air_tank',cyl(105,570,(-1990,-285,665),(0,1,0)),'silver')   # above the fuel cell
    add('Air_compressors',[box(170,100,100,(-1990,s*440,410)) for s in [-1,1]],'black')

    print('Building Hellcat / Demon family drivetrain',flush=True)
    # Engine local frame: origin on the crank centerline at the block's rear (bellhousing) face,
    # +X forward, Z up. Every engine/trans part is built here, then tilted and placed as one unit,
    # so changing DT moves everything together.
    L=DT['block_length']; t=math.radians(DT['tilt_deg'])
    bell=(DT['block_front_x']-L*math.cos(t),0,DT['crank_z_front']-L*math.sin(t))
    def place(shape):return shape.rotate(cq.Vector(0,0,0),cq.Vector(0,1,0),-DT['tilt_deg']).translate(bell)
    def to_world(x,y,z):
        return (bell[0]+x*math.cos(t)-z*math.sin(t),y,bell[2]+x*math.sin(t)+z*math.cos(t))
    def yz_extrude(poly,x0,x1):
        return cq.Solid.extrudeLinear(wire([(x0,y,z) for y,z in poly]),[],cq.Vector(x1-x0,0,0))
    def mirror_poly(half):  # half = right side points from bottom centre to top centre
        return half+[(-y,z) for y,z in reversed(half)]
    def bank(d0,d1,w,s):
        """Rectangle in the front view, standing on a 45-degree bank (90-degree V)."""
        ax=(s*math.sin(math.radians(DT['bank_angle'])),math.cos(math.radians(DT['bank_angle'])))
        pe=(ax[1],-ax[0]) if s>0 else (-ax[1],ax[0])
        c=lambda d,k:(ax[0]*d+pe[0]*k,ax[1]*d+pe[1]*k)
        return [c(d0,-w/2*s),c(d0,w/2*s),c(d1,w/2*s),c(d1,-w/2*s)]
    deck=DT['deck_height']; hw=DT['skirt_half_width']
    blk=yz_extrude([(-hw,-95),(hw,-95),(hw+10,40)]+[bank(deck,deck,DT['head_width']-20,1)[i] for i in (1,0)]
                   +[(40,175),(-40,175)]+[bank(deck,deck,DT['head_width']-20,-1)[i] for i in (1,0)]+[(-hw-10,40)],0,L)
    heads=[];covers=[];ports=[]
    for s in [-1,1]:
        stagger=DT['bank_stagger']*(0.5 if s>0 else -0.5)
        heads.append(yz_extrude(bank(deck,deck+DT['head_height'],DT['head_width'],s),8+stagger,L-8+stagger))
        top=deck+DT['head_height']
        covers.append(yz_extrude(bank(top,top+DT['cover_height'],DT['cover_width'],s),30+stagger,L-30+stagger))
        for k in range(4):
            cx=L-70-k*DT['bore_spacing']+stagger
            p=bank(deck+40,deck+40,DT['head_width'],s)[1 if s>0 else 0]
            ports.append((cx,p[0]+s*12,p[1]-8,s))
    pan=[yz_extrude([(-150,-95),(150,-95),(140,-95-DT['pan_front_depth']),(-140,-95-DT['pan_front_depth'])],DT['sump_length'],L-15),
         yz_extrude([(-165,-95),(165,-95),(155,-95-DT['sump_depth']),(-155,-95-DT['sump_depth'])],12,DT['sump_length'])]
    front_cover=yz_extrude(mirror_poly([(110,-110),(150,-60),(170,120),(90,250)]),L,L+45)
    bh=DT['blower_height']
    blower_poly=mirror_poly([(75,170),(225,280),(225,330),(165,350),(165,bh-15),(150,bh)])
    blower=yz_extrude(blower_poly,50,L-20)
    lid=yz_extrude(mirror_poly([(170,bh-30),(170,bh-12),(150,bh+4)]),62,L-40)
    lid_ribs=[box(L-110,7,8,(L/2,y,bh+2)) for y in range(-120,121,30)]
    belt_x=L+110
    pulleys_spec=[('crank',0,0,DT['crank_pulley_r']),('blower',0,DT['blower_pulley_z'],DT['blower_pulley_r']),
                  ('water_pump',0,175,68),('alternator',-235,215,38),('idler_rh',215,205,45),('tensioner',-120,340,34)]
    pulleys=[cyl(r,30,(belt_x-15,y,z),(1,0,0)) for _,y,z,r in pulleys_spec]
    snout=cyl(42,belt_x-15-(L-20),(L-20,0,DT['blower_pulley_z']),(1,0,0))
    balancer=cyl(112,belt_x-15-(L+45),(L+45,0,0),(1,0,0))
    alt_body=cyl(70,150,(L-90,-235,215),(1,0,0))
    belt_order=['crank','alternator','tensioner','blower','idler_rh','water_pump','crank']
    bp={n:(y,z) for n,y,z,r in pulleys_spec}
    belt=[tube((belt_x,*bp[a]),(belt_x,*bp[b]),7) for a,b in zip(belt_order,belt_order[1:])]
    tb_z=DT['throttle_body_z']
    tbody=cyl(DT['throttle_body_r'],90,(L-30,0,tb_z),(1,0,0))
    # 8-speed automatic: bellhousing cone, main case, tail housing (truncated cones on the crank line).
    tr=DT['trans']
    def cone(x0,x1,r0,r1):return cq.Solid.makeCone(r0,r1,x0-x1,cq.Vector(x0,0,0),cq.Vector(-1,0,0))
    xb=-tr['bell_length'];xc=xb-tr['case_length'];xt=xc-tr['tail_length']
    trans=[cone(0,xb,tr['bell_r_front'],tr['bell_r_rear']),cone(xb,xc,tr['case_r_front'],tr['case_r_rear']),
           cone(xc,xt,tr['tail_r'],tr['tail_r']*.8),
           box(tr['case_length']*.7,tr['case_r_front']*1.3,45,(xb-tr['case_length']*.45,0,-tr['case_r_front']-5))]
    flexplate_cover=cyl(tr['bell_r_front']+6,10,(-10,0,0),(1,0,0))
    # Motor mounts: block ears to frame-rail pads (rubber biscuit + bracket).
    mounts=[];mount_pads=[]
    mx=L*.62
    for s in [-1,1]:
        mounts.append(yz_extrude([(s*(hw-5),20),(s*(hw+75),-20),(s*(hw+75),-75),(s*(hw-5),-60)],mx-40,mx+40))
    add('Hellcat_engine_block',[place(blk),place(front_cover)]+[place(p) for p in pan],'black')
    add('Hellcat_cylinder_heads',[place(h) for h in heads],'silver')
    add('Hellcat_valve_covers',[place(c) for c in covers],'black')
    add('Supercharger_and_intake',[place(blower),place(lid)],'silver')
    add('Supercharger_lid_ribs',[place(r) for r in lid_ribs],'black')
    add('Throttle_body',place(tbody),'silver')
    add('Accessory_pulleys',[place(p) for p in pulleys]+[place(snout),place(balancer),place(alt_body)],'black')
    add('Belt_path',[place(b) for b in belt],'tire')
    add('Eight_speed_automatic',[place(p) for p in trans]+[place(flexplate_cover)],'silver')
    add('Engine_mount_brackets',[place(m) for m in mounts],'silver')
    # Mount pads: from each bracket straight out to the frame rail inner face.
    for s in [-1,1]:
        a=to_world(mx,s*(hw+75),-45); rail_y=480-40
        mount_pads.append(box(80,abs(rail_y-abs(a[1]))+12,60,(a[0],s*(abs(a[1])+rail_y)/2,a[2])))
    add('Engine_mount_rubber_pads',mount_pads,'tire')
    # Headers: four primaries per side from the outboard exhaust ports, down to a collector beside the bell.
    headers=[];collectors={}
    for s in [-1,1]:
        coll=(DT['collector_x'],s*DT['collector_y'],DT['collector_z'])
        collectors[s]=coll
        for cx,py,pz,_ in [p for p in ports if p[3]==s]:
            a=to_world(cx,py,pz); b=(a[0]-25,s*(abs(a[1])+55),a[2]-60); c=(a[0]-60,s*DT['collector_y'],DT['collector_z']+150)
            headers += [tube(a,b,25.4),tube(b,c,25.4),tube(c,coll,25.4)]   # 2 in primaries
        headers.append(tube(coll,(coll[0]-110,coll[1],coll[2]-8),44.45))   # 3.5 in collector
    add('Long_tube_headers',headers,'silver')
    # Air path: throttle body -> elbow -> tube -> filter box in the driver-side front corner.
    tb_front=to_world(L+60,0,tb_z)
    elbow=(tb_front[0]+20,-170,tb_front[2]+15); box_c=DT['airbox_center']
    add('Cold_air_intake',[tube(tb_front,elbow,DT['throttle_body_r']+4),tube(elbow,(box_c[0]-40,box_c[1]+90,box_c[2]),DT['throttle_body_r']+4),
                           box(*DT['airbox_size'],box_c)],'black')
    # Driveline: trans output -> prop shaft -> pinion, plus a solid rear axle center section.
    out=to_world(xt,0,0); rear_z=P['rear_tire_diameter']/2
    pinion_len=DT['pinion_length']; pa=math.radians(DT['tilt_deg'])  # pinion set parallel to the crank line
    yoke=(-1400+pinion_len*math.cos(pa),0,rear_z-SUSP['nine_pinion_drop']+pinion_len*math.sin(pa))
    add('Prop_shaft',[tube(out,yoke,DT['prop_r']),cyl(DT['prop_r']+12,40,(out[0]-20,0,out[2]),(1,0,0)),cyl(DT['prop_r']+12,40,(yoke[0]-20,0,yoke[2]),(1,0,0))],'black')
    # Ford 9-inch: big banjo center, drop-out third member pointing forward, 3 in tubes, big-bearing ends.
    tr9=SUSP['nine_tube_od']/2; pin_z=rear_z-SUSP['nine_pinion_drop']
    banjo=cyl(165,190,(-1400,-95,rear_z),(0,1,0))
    third=cq.Solid.makeCone(120,58,yoke[0]-(-1400+90),cq.Vector(-1400+90,0,pin_z),cq.Vector(1,0,0))
    flange=cyl(150,22,(-1400+80,0,rear_z-20),(1,0,0))
    add('Ford_9in_center_section',[banjo,third,flange],'frame')
    add('Ford_9in_axle_tubes',[tube((-1400,s*90,rear_z),(-1400,s*720,rear_z),tr9) for s in [-1,1]]+[cyl(62,22,(-1400,s*(720 if s>0 else 742),rear_z),(0,1,0)) for s in [-1,1]],'silver')
    # Trans crossmember: steps down under the tail housing between the exhaust runs; rubber mount on top.
    tm=to_world(xc-40,0,-tr['tail_r']); cz=tm[2]-50
    tcm=[(tm[0],-570,285),(tm[0],-200,285),(tm[0],-90,cz),(tm[0],90,cz),(tm[0],200,285),(tm[0],570,285)]
    add('Trans_crossmember',[boxtube(a,b,FRAME['cross']) for a,b in zip(tcm,tcm[1:])],'frame')
    add('Trans_mount_rubber',box(110,140,tm[2]-cz-38+4,(tm[0],0,(tm[2]+cz+38)/2)),'tire')
    DRIVELINE.update(trans_output=out,pinion_yoke=yoke,trans_mount_x=tm[0])

    add('Radiator',[box(65,1040,420,(1940,0,532)),box(140,1050,45,(1980,0,758))],'black')
    add('Electric_fans_and_shroud',[box(40,900,400,(1882,0,532))]+[cyl(70,55,(1810,y,532),(1,0,0)) for y in [-220,220]],'black')
    exhaust=[]
    for s in [-1,1]:
        c=collectors[s]
        # Inboard of the rear suspension pivots, over the diffuser, out through notches in the rear valance.
        pts=[(c[0]-110,c[1],c[2]-8),(820,s*330,245),(300,s*330,DT['exhaust_z']),(-950,s*300,DT['exhaust_z']),(-1200,s*292,500),(-1560,s*292,505),(-1730,s*300,290),(-2200,s*420,284),(-2478,s*440,294)]
        exhaust += [tube(a,b,44.45) for a,b in zip(pts,pts[1:])]   # dual 3.5 in
    add('Dual_exhaust',exhaust,'silver')
    add('Fuel_cell',box(500,770,200,(-2000,0,458)),'frame')

    print('Building firewall, floor, tunnel and interior',flush=True)
    # Tunnel sections follow the drivetrain: clearance over the trans, then the prop shaft line.
    def axis(x):
        if x>=out[0]:
            return bell[2]+(x-bell[0])*math.tan(t)
        return out[2]+(x-out[0])*(yoke[2]-out[2])/(yoke[0]-out[0])
    def radius_at(x):
        lx=(x-bell[0])/math.cos(t)
        if lx>xb: return tr['bell_r_rear']+(tr['bell_r_front']-tr['bell_r_rear'])*(lx-xb)/(0-xb)
        if lx>xc: return tr['case_r_rear']+(tr['case_r_front']-tr['case_r_rear'])*(lx-xc)/(xb-xc)
        if lx>xt: return tr['tail_r']
        return DT['prop_r']+25   # U-joint swing room
    gap=DT['tunnel_clearance']
    tunnel_x=[DT['firewall_x']+5]+list(range(int(DT['firewall_x'])-35,-850,-50))+[-850]
    def arch(x,extra=0):
        h=max(max(radius_at(x+d) for d in (-30,0,30))+gap,110)+extra; zc=max(axis(x+d) for d in (-30,0,30)); floor_z=DT['floor_z']
        pts=[(x,-h,floor_z-30),(x,-h,zc)]+[(x,-h*math.cos(a),zc+h*math.sin(a)) for a in [.5,1.0,1.3]]+[(x,0,zc+h)]
        return pts+[(x,-px,pz) for x_,px,pz in reversed(pts[1:-1])]+[(x,h,floor_z-30)]
    inner_x=tunnel_x
    # Sheet-metal tunnel skin: open arch sections lofted into one surface sitting on the floor.
    ts=BRepOffsetAPI_ThruSections(False,True,1e-6)
    for x in tunnel_x:
        pts=arch(x,0)[1:-1]
        ts.AddWire(cq.Wire.makePolygon([cq.Vector(p[0],p[1],p[2]) for p in [(x,pts[0][1],DT['floor_z'])]+pts+[(x,pts[-1][1],DT['floor_z'])]],close=False).wrapped)
    ts.Build();tunnel=cq.Shape.cast(ts.Shape())
    add('Transmission_tunnel',tunnel,'black')
    hw=lambda x:abs(arch(x,0)[0][1])
    foot=[(x,-hw(x),DT['floor_z']-20) for x in inner_x]+[(x,hw(x),DT['floor_z']-20) for x in reversed(inner_x)]
    tunnel_hole=cq.Solid.extrudeLinear(wire(foot),[],cq.Vector(0,0,80))
    floor=box(1550,1420,25,(-75,0,DT['floor_z']+12.5)).cut(tunnel_hole)   # ends at x=-850, at the rear crossmember
    add('Cabin_floor',floor,'black')
    fw_hole=cq.Solid.extrudeLinear(wire([(DT['firewall_x']+30,y,z) for _,y,z in arch(DT['firewall_x'],0)]),[],cq.Vector(-60,0,0))
    fw=box(12,1400,520,(DT['firewall_x'],0,DT['floor_z']+260)).cut(fw_hole)
    add('Firewall',fw,'black')
    seats=[]
    for s in [-1,1]:
        seats += [box(465,435,110,(-200,s*410,427)),box(130,450,500,(-460,s*410,700)).rotate((-460,0,700),(-460,1,700),-12),box(140,270,170,(-490,s*410,1006))]
    add('Sport_bucket_seats',seats,'black')
    add('Dashboard',box(210,1390,155,(720,0,840)),'black')
    add('Steering_wheel',ring(160,143,22,(555,380,859)).rotate((555,380,859),(555,380,860),90),'black')
    console_z=max(arch(x,0)[len(arch(x,0))//2][2] for x in range(-410,400,20))+6
    add('Center_console',box(800,220,150,(-10,0,console_z+75)),'frame')

    # Hood clearance: cut a pocket through the hood under the scoop so the blower lives inside the scoop.
    # The scoop shell keeps its outside shape; DT['scoop_wall'] sets how thick the shell stays.
    pocket_sections=[]
    for x,w,h in scoop_stations[1:-1]:
        ww=w*DT['scoop_pocket_width_frac']
        top=[(x,ww*f,hood_top(x,ww*f)+(h-DT['scoop_wall'])*rise) for f,rise in
             [(-1,.15),(-.96,.25),(-.83,.62),(-.57,.92),(0,1),(.57,.92),(.83,.62),(.96,.25),(1,.15)]]
        pocket_sections.append([(x,-ww,DT['hood_pocket_floor_z'])]+top+[(x,ww,DT['hood_pocket_floor_z'])])
    pocket=fair_body_loft(pocket_sections)
    for name in ['Hood','Blended_hood_scoop']:
        shapes,col,cat=PARTS[name];PARTS[name]=([s.cut(pocket) for s in shapes],col,cat)
    return PARTS

DRIVELINE={};BAGS=[];MINZ={}
UNSPRUNG={'Tires','Wheel_barrels_and_lips','Forged_split_spokes','Wheel_lugs','Brake_rotors','Brake_calipers',
          'Ford_9in_center_section','Ford_9in_axle_tubes','Rear_bag_lower_plates','Rear_4link_axle_brackets'}
def ride_meta():
    """How each part moves when the bags go up or down. mode 0 = stays with the wheels, 1 = moves with the body,
    2 = stretches across Y (arms), 3 = bag stretches in Z, 4 = prop shaft stretches along X."""
    lo_f,hi_f,wlo_f=BAGS[0];lo_r,hi_r,_=BAGS[-1]
    o,y=DRIVELINE['trans_output'],DRIVELINE['pinion_yoke']
    modes={}
    for n in PARTS:
        if n in UNSPRUNG:modes[n]=[0,0,0,0,0]
        elif n in ('Front_lower_arms','Front_bag_lower_plates'):modes[n]=[2,745,SUSP['f_lower_pivot_y'],0,0]
        elif n=='Front_upper_arms':modes[n]=[2,735,SUSP['f_upper_pivot_y'],0,0]
        elif n=='Rear_4link_lower_bars':modes[n]=[4,SUSP['r_lower_frame_x'],-1400,0,0]
        elif n=='Rear_4link_upper_bars':modes[n]=[4,SUSP['r_upper_frame_x'],-1400,0,0]
        elif n=='Air_springs':modes[n]=[3,lo_f,hi_f,lo_r,hi_r,wlo_f]
        elif n=='Prop_shaft':modes[n]=[4,o[0],y[0],0,0]
        else:modes[n]=[1,0,0,0,0]
    inch=25.4
    return dict(modes=modes,lowest_sprung_z=round(MINZ['Rockers_and_panel_lines'],1),
                states={'Laid out':LAID_DZ,'Drive':LAID_DZ+RIDE['drive_above_laid_in']*inch,'Lifted':LAID_DZ+RIDE['lift_above_laid_in']*inch})
def ride_weight(mode,p,v):
    m=mode
    if m==0:return 0.0
    if m==1:return 1.0
    if m==2:return min(max((p[0]-abs(v[1]))/(p[0]-p[1]),0),1)
    if m==3:
        lo,hi,wlo=(p[0],p[1],p[4]) if v[0]>0 else (p[2],p[3],0.0)
        return wlo+(1-wlo)*min(max((v[2]-lo)/(hi-lo),0),1)
    if m==4:return min(max((v[0]-p[1])/(p[0]-p[1]),0),1)
def ride_states_check(meshdata):
    """Every moving part (wheels, brakes, arms, 4-link, bags, 9-inch, driveshaft) against everything,
    at laid out, drive and lifted. Returns rows of contacts / tight spots."""
    import fcl, numpy as np
    RM=ride_meta();modes=RM['modes']
    wheel={'Tires','Wheel_barrels_and_lips','Forged_split_spokes','Brake_rotors','Brake_calipers','Wheel_lugs'}
    moving=list(wheel)+['Front_lower_arms','Front_upper_arms','Rear_4link_lower_bars','Rear_4link_upper_bars','Air_springs',
            'Front_bag_lower_plates','Rear_bag_lower_plates','Ford_9in_center_section','Ford_9in_axle_tubes','Rear_4link_axle_brackets','Prop_shaft']
    ok_pairs={frozenset(p) for p in [('Front_lower_arms','Front_subframe_and_arm_towers'),('Front_upper_arms','Front_subframe_and_arm_towers'),
        ('Front_lower_arms','Front_bag_lower_plates'),('Front_lower_arms','Bump_stops'),('Air_springs','Front_bag_lower_plates'),
        ('Air_springs','Rear_bag_lower_plates'),('Air_springs','Air_spring_upper_plates'),('Rear_4link_lower_bars','Rear_4link_axle_brackets'),
        ('Rear_4link_upper_bars','Rear_4link_axle_brackets'),('Rear_4link_lower_bars','Rear_4link_frame_brackets'),
        ('Rear_4link_upper_bars','Rear_4link_frame_brackets'),('Rear_4link_axle_brackets','Ford_9in_axle_tubes'),
        ('Rear_bag_lower_plates','Ford_9in_axle_tubes'),('Ford_9in_axle_tubes','Bump_stops'),('Ford_9in_center_section','Bump_stops'),('Ford_9in_center_section','Ford_9in_axle_tubes'),
        ('Prop_shaft','Ford_9in_center_section'),('Prop_shaft','Eight_speed_automatic'),('Rear_4link_axle_brackets','Ford_9in_center_section'),
        ('Front_bag_lower_plates','Front_subframe_and_arm_towers'),('Rear_4link_lower_bars','Ford_9in_axle_tubes'),('Brake_rotors','Ford_9in_axle_tubes'),
        ('Air_springs','Custom_box_frame'),('Air_springs','Front_subframe_and_arm_towers'),('Rear_4link_upper_bars','Ford_9in_axle_tubes'),('Air_spring_upper_plates','Custom_box_frame'),
        ('Rear_4link_lower_bars','Custom_box_frame'),('Rear_4link_upper_bars','Custom_box_frame')]}
    base={n:(np.array(c,float),np.array(f,np.int32)) for n,col,cat,c,f in meshdata}
    def placed(n,dz):
        V,F=base[n];md=modes[n]
        if md[0]==0 or dz==0:W=0.0
        elif md[0]==1:W=1.0
        else:W=np.array([ride_weight(md[0],md[1:],v) for v in V])
        V2=V.copy();V2[:,2]+=W*dz
        m=fcl.BVHModel();m.beginModel(len(V2),len(F));m.addSubModel(V2,F);m.endModel()
        return fcl.CollisionObject(m,fcl.Transform()),V2.min(0),V2.max(0)
    rows=[]
    for state,dz in RM['states'].items():
        G={n:placed(n,dz) for n in base}
        seen=set()
        for a in moving:
            for b in base:
                if a==b or frozenset((a,b)) in seen or frozenset((a,b)) in ok_pairs or (a in wheel and b in wheel):continue
                seen.add(frozenset((a,b)))
                oa,amin,amax=G[a];ob,bmin,bmax=G[b]
                if np.any(amax+40<bmin) or np.any(bmax+40<amin):continue
                if fcl.collide(oa,ob,fcl.CollisionRequest(),fcl.CollisionResult()):g=0.0
                else:g=fcl.distance(oa,ob,fcl.DistanceRequest(),fcl.DistanceResult())
                if g<DT['min_clearance']:
                    rows.append(dict(state=state,part=a,against=b,gap_mm=round(g,1),status='CLASH / contact' if g<=0.5 else 'tight'))
    return rows
def fit_check(out_dir,meshdata):
    """Measure drivetrain and aero parts against everything around them, using the exported meshes
    (accurate to the ~3 mm tessellation tolerance). Writes fit_report.json."""
    import fcl, numpy as np
    print('Running fit check',flush=True)
    moving=['Hellcat_engine_block','Hellcat_cylinder_heads','Hellcat_valve_covers','Supercharger_and_intake','Supercharger_lid_ribs',
            'Throttle_body','Accessory_pulleys','Belt_path','Eight_speed_automatic','Long_tube_headers','Cold_air_intake',
            'Prop_shaft','Ford_9in_center_section','Ford_9in_axle_tubes','Dual_exhaust','Front_air_dam','Front_splitter']
    around=['Body_shell','Hood','Blended_hood_scoop','Custom_box_frame','Trans_crossmember','Radiator','Electric_fans_and_shroud',
            'Firewall','Transmission_tunnel','Cabin_floor','Center_console','Dashboard','Cage_windshield_and_A_legs','Cage_sill_rails_and_chassis_ties',
            'Suspension_control_arms','Air_springs','Air_spring_upper_plates','Shock_absorbers','Brake_rotors','Tires','Fuel_cell','Air_tank',
            'Air_compressors','Headlights','Recessed_headlight_buckets','Fog_light_recessed_buckets','Rear_diffuser','Long_tube_headers','Dual_exhaust']
    # Joints that are supposed to touch (bolted / welded / sealed), plus exhaust sleeved through crossmembers.
    intended={('Long_tube_headers','Dual_exhaust'),('Dual_exhaust','Rear_diffuser'),('Front_air_dam','Body_shell'),
              ('Front_splitter','Front_air_dam'),('Ford_9in_axle_tubes','Air_spring_lower_plates'),
              ('Ford_9in_axle_tubes','Ford_9in_center_section'),('Prop_shaft','Ford_9in_center_section'),
              ('Prop_shaft','Eight_speed_automatic'),('Front_splitter','Body_shell'),('Dual_exhaust','Custom_box_frame'),
              ('Dual_exhaust','Trans_crossmember'),('Hellcat_engine_block','Eight_speed_automatic'),('Hellcat_cylinder_heads','Long_tube_headers'),
              ('Ford_9in_axle_tubes','Shock_absorbers'),('Ford_9in_center_section','Custom_box_frame'),('Ford_9in_axle_tubes','Bump_stops'),('Ford_9in_center_section','Bump_stops'),('Ford_9in_axle_tubes','Brake_rotors')}
    geo={}
    for name,color,cat,coords,faces in meshdata:
        v=np.array(coords,dtype=float);f=np.array(faces,dtype=np.int32)
        m=fcl.BVHModel();m.beginModel(len(v),len(f));m.addSubModel(v,f);m.endModel()
        geo[name]=(m,v.min(0),v.max(0))
    def dist(a,b,dz=0.0):
        A,amin,amax=geo[a];B,bmin,bmax=geo[b]
        bmin=bmin+[0,0,dz];bmax=bmax+[0,0,dz]
        if np.any(amax+80<bmin) or np.any(bmax+80<amin):return None
        oa=fcl.CollisionObject(A,fcl.Transform());ob=fcl.CollisionObject(B,fcl.Transform(np.array([0,0,dz],dtype=float)))
        if fcl.collide(oa,ob,fcl.CollisionRequest(),fcl.CollisionResult()):return 0.0
        return fcl.distance(oa,ob,fcl.DistanceRequest(),fcl.DistanceResult())
    def status(a,b,g):
        if (a,b) in intended or (b,a) in intended:return 'intended contact'
        return 'CLASH / contact' if g<=0.5 else ('tight' if g<DT['min_clearance'] else 'ok')
    rows=[];seen=set()
    for a in moving:
        for b in around:
            if a==b or (b,a) in seen or a not in geo or b not in geo:continue
            seen.add((a,b));g=dist(a,b)
            if g is None or g>DT['report_gap_under']:continue
            rows.append(dict(part=a,against=b,gap_mm=round(g,1),status=status(a,b,g)))
    order={'CLASH / contact':0,'tight':1,'ok':2,'intended contact':3}
    rows.sort(key=lambda r:(order[r['status']],r['gap_mm']))
    RM=ride_meta();dz=RM['states']['Laid out'];laid_rows=[]
    for a in ['Ford_9in_center_section','Ford_9in_axle_tubes','Tires','Brake_calipers','Brake_rotors']:
        for b in ['Body_shell','Hood','Molded_integrated_fender_flares','Bump_stops','Custom_box_frame','Fuel_cell','Dual_exhaust','Air_tank','Air_compressors','Rear_diffuser','Trans_crossmember',
                  'Cabin_floor','Transmission_tunnel','Long_tube_headers','Hellcat_engine_block','Cold_air_intake','Radiator','Front_air_dam','Front_splitter']:
            g=dist(a,b,dz)
            if g is not None and g<=DT['report_gap_under']:
                laid_rows.append(dict(part=a,against=b+' (laid out)',gap_mm=round(g,1),status=status(a,b,g)))
    laid_rows.sort(key=lambda r:r['gap_mm'])
    low={n:round(float(geo[n][1][2])+dz,1) for n in ['Rockers_and_panel_lines','Front_splitter','Body_shell','Dual_exhaust','Hellcat_engine_block',
                                             'Eight_speed_automatic','Custom_box_frame','Trans_crossmember','Rear_diffuser'] if n in geo}
    o,y=DRIVELINE['trans_output'],DRIVELINE['pinion_yoke']
    def prop_angle(d):return math.degrees(math.atan2(y[2]-(o[2]+d),o[0]-y[0]))
    ride_angles={k:dict(rocker_height_in=round((RM['lowest_sprung_z']+v)/25.4,1),prop_deg_up_toward_rear=round(prop_angle(v),2),
                        ujoint_working_deg=round(DT['tilt_deg']+prop_angle(v),2)) for k,v in RM['states'].items()}
    MOV=ride_states_check(meshdata)
    for r_ in MOV:print(f"  {r_['state']:9s} {r_['status']:16s} {r_['part']:26s} vs {r_['against']:34s} gap {r_['gap_mm']:6.1f}")
    print('  moving-part problems:',len(MOV))
    report=dict(method='mesh distance (FCL), accurate to about 3 mm',min_clearance_target_mm=DT['min_clearance'],
                lowest_points_laid_out_mm=low,ride_states=ride_angles,
                note_driveline='Pinion is set parallel to the crank line at drive height; front and rear U-joint angles match.',
                drive_height_checks=rows,laid_out_checks=laid_rows,moving_parts_all_ride_heights=MOV)
    (out_dir/'fit_report.json').write_text(json.dumps(report,indent=2))
    for r in rows+laid_rows:
        if r['status']!='ok':print(f"  {r['status']:16s} {r['part']:28s} vs {r['against']:36s} gap {r['gap_mm']:6.1f}")
    print('  ok pairs:',sum(r['status']=='ok' for r in rows+laid_rows));print('  lowest when laid out:',low)
    for r in laid_rows:print('   laid:',r)
    for k,v in ride_angles.items():print('  ',k,v)
    return report

def export():
    out=ROOT/'output';out.mkdir(exist_ok=True)
    asm=cq.Assembly(name='CUDA_CAT_170_REV_B')
    payload=bytearray(); meta=[]; meshdata=[]; report=[]
    for name,(shapes,color,category) in PARTS.items():
        solid=cq.Compound.makeCompound(shapes)
        if not solid.isValid(): raise ValueError('Invalid solid: '+name)
        bb=solid.BoundingBox()
        if max(abs(bb.xmin),abs(bb.xmax),abs(bb.ymin),abs(bb.ymax),abs(bb.zmin),abs(bb.zmax))>6000: raise ValueError('Part blew up in size: '+name)
        asm.add(solid,name=name,color=cq.Color(*color))
        vs,fs=solid.tessellate(3,0.25)
        coords=[[v.x,v.y,v.z] for v in vs];MINZ[name]=min(c[2] for c in coords)
        meta.append([name,color,len(vs),len(fs),category]);meshdata.append((name,color,category,coords,fs))
        old=[0,0,0]
        for v in coords:
            vals=[round(n*2) for n in v];payload.extend(struct.pack('<hhh',*(a-b for a,b in zip(vals,old))));old=vals
        prev=0
        for f in fs:
            for i in f:payload.extend(struct.pack('<i',i-prev));prev=i
        report.append(dict(name=name,category=category,valid=True,solids=len(solid.Solids()),triangles=len(fs)))
    print('Exporting STEP assembly',flush=True)
    asm.export(str(out/'cuda_cat_170_revB.step'))
    cq.exporters.export(cq.Compound.makeCompound([s for ss,c,g in PARTS.values() for s in ss]),str(out/'cuda_cat_170_revB.stl'),tolerance=3,angularTolerance=.25)
    (out/'mesh.json').write_text(json.dumps(meshdata))
    RM=ride_meta();(out/'ride.json').write_text(json.dumps(RM,indent=1))
    (out/'validation.json').write_text(json.dumps(dict(parameters=P,drivetrain=DT,aero=AERO,parts=report),indent=2))
    data=base64.b64encode(gzip.compress(payload)).decode()
    template=(ROOT/'viewer_template.html').read_text()
    html=template.replace('__RIDE__',json.dumps(RM)).replace('__TITLE__','Cuda-Cat 170 — Rev B').replace('__META__',json.dumps(meta)).replace('__DATA__',data)
    (out/'cuda_cat_170_revB_3d.html').write_text(html,encoding='utf-8')
    print(f'Exported {len(PARTS)} groups / {sum(p[3] for p in meta):,} triangles',flush=True)
    return meshdata

# ---- Show the car in CQ-editor ----
build()
for _name,(_shapes,_color,_cat) in PARTS.items():
    _opts={"color":tuple(int(c*255) for c in _color)}
    if 'glass' in _name.lower() or _name=='Windshield':_opts["alpha"]=0.6
    try:
        show_object(cq.Compound.makeCompound(_shapes),name=_name,options=_opts)
    except NameError:     # not running inside CQ-editor
        pass
print('Shown',len(PARTS),'parts')
