"""Offscreen VTK renders from output/mesh.json (+ ride.json). Usage: python render.py <outdir>"""
import sys, json, pathlib
import vtk
out = pathlib.Path(sys.argv[1])
mesh = json.loads((out / 'mesh.json').read_text())
ride = json.loads((out / 'ride.json').read_text()) if (out / 'ride.json').exists() else None
def weight(m, p, v):
    if m == 0: return 0.0
    if m == 1: return 1.0
    if m == 2: w = (p[0]-abs(v[1]))/(p[0]-p[1])
    elif m == 3:
        lo, hi, wl = (p[0], p[1], (p[4] if len(p) > 4 else 0)) if v[0] > 0 else (p[2], p[3], 0)
        w = wl + (1-wl)*min(max((v[2]-lo)/(hi-lo), 0), 1)
    else: w = (v[0]-p[1])/(p[0]-p[1])
    return min(max(w, 0), 1)
SHOTS = [  # (file, state, camera pos, focal, hide hood?, parallel scale or None)
    ('hero_front34', 'Drive', (5200, 3900, 1500), (0, 0, 560), False, None),
    ('hero_rear34', 'Drive', (-5300, 3700, 1700), (0, 0, 600), False, None),
    ('laid_front34', 'Laid out', (5200, 3900, 1300), (0, 0, 420), False, None),
    ('side_laid', 'Laid out', (0, 8200, 650), (0, 0, 650), False, 1750),
    ('side_drive', 'Drive', (0, 8200, 650), (0, 0, 650), False, 1750),
    ('side_lifted', 'Lifted', (0, 8200, 650), (0, 0, 650), False, 1750),
    ('engine_bay', 'Drive', (3500, 2400, 2500), (1250, 0, 600), True, None),
    ('top', 'Drive', (0, 0, 9000), (0, 0, 0), False, 2700),
]
def make_actors(ren, state, hide_hood):
    dz = ride['states'][state] if ride else 0
    for name, color, category, coords, faces in mesh:
        if hide_hood and category == 'hood': continue
        md = ride['modes'].get(name, [1, 0, 0, 0, 0]) if ride else [1, 0, 0, 0, 0]
        pts = vtk.vtkPoints()
        for c in coords:
            w = weight(md[0], md[1:], c) if dz else 0
            pts.InsertNextPoint(c[0], c[1], c[2] + w*dz)
        polys = vtk.vtkCellArray()
        for f in faces:
            polys.InsertNextCell(3)
            for i in f: polys.InsertCellPoint(i)
        pd = vtk.vtkPolyData(); pd.SetPoints(pts); pd.SetPolys(polys)
        nm = vtk.vtkPolyDataNormals(); nm.SetInputData(pd); nm.SetFeatureAngle(35); nm.Update()
        m = vtk.vtkPolyDataMapper(); m.SetInputConnection(nm.GetOutputPort())
        a = vtk.vtkActor(); a.SetMapper(m); p = a.GetProperty(); p.SetColor(*color)
        p.SetAmbient(.12); p.SetDiffuse(.9)
        p.SetSpecular(.6 if category in ('body', 'hood') else .3); p.SetSpecularPower(55)
        if 'glass' in name.lower() or name == 'Windshield': p.SetOpacity(.55)
        ren.AddActor(a)
    # ground shadow plane
    plane = vtk.vtkPlaneSource(); plane.SetOrigin(-30000, -30000, -1); plane.SetPoint1(30000, -30000, -1); plane.SetPoint2(-30000, 30000, -1)
    pm = vtk.vtkPolyDataMapper(); pm.SetInputConnection(plane.GetOutputPort()); pa = vtk.vtkActor(); pa.SetMapper(pm)
    pa.GetProperty().SetColor(.86, .87, .89); pa.GetProperty().SetAmbient(.5); pa.GetProperty().SetDiffuse(.3); ren.AddActor(pa)
for fname, state, pos, foc, hide, scale in SHOTS:
    if not ride and state != 'Drive': continue
    ren = vtk.vtkRenderer(); ren.SetBackground(.96, .965, .97); ren.SetBackground2(.80, .82, .85); ren.GradientBackgroundOn()
    make_actors(ren, state, hide)
    cam = ren.GetActiveCamera(); cam.SetPosition(*pos); cam.SetFocalPoint(*foc); cam.SetViewUp(1 if fname == 'top' else 0, 0, 0 if fname == 'top' else 1)
    cam.SetViewAngle(28)
    if scale: cam.ParallelProjectionOn(); cam.SetParallelScale(scale)
    ren.ResetCameraClippingRange()
    ren.RemoveAllLights()
    for p, i in [((4000, 3000, 5000), .85), ((-3000, 4000, 2500), .45), ((2000, -4000, 3000), .35), ((0, 0, -3000), .1)]:
        l = vtk.vtkLight(); l.SetLightTypeToSceneLight(); l.SetPosition(*p); l.SetFocalPoint(0, 0, 400); l.SetIntensity(i); ren.AddLight(l)
    if hasattr(ren, 'UseShadowsOff'): ren.UseShadowsOff()
    win = vtk.vtkRenderWindow(); win.SetOffScreenRendering(1); win.SetSize(1800, 1050); win.SetMultiSamples(8); win.AddRenderer(ren); win.Render()
    w2i = vtk.vtkWindowToImageFilter(); w2i.SetInput(win); w2i.Update()
    wr = vtk.vtkPNGWriter(); wr.SetFileName(str(out / f'render_{fname}.png')); wr.SetInputConnection(w2i.GetOutputPort()); wr.Write()
    print('wrote', fname)
