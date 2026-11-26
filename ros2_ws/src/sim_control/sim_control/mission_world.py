"""Render a temporary mission world from the checked YAML, preserving textures."""

from pathlib import Path
import xml.etree.ElementTree as ET


def render_world(source, target, cfg, *, include_racks=True):
    source = Path(source).resolve()
    root = ET.parse(source).getroot()
    world = root.find('world')
    models = {m.get('name'): m for m in world.findall('model')}
    # Temp files live outside the package: retain absolute texture/mesh paths.
    for tag in ('uri', 'albedo_map', 'emissive_map'):
        for node in root.iter(tag):
            if node.text and '://' not in node.text:
                node.text = str((source.parent / node.text).resolve())
    scene = cfg['scene']
    f = scene['field']
    xmin,xmax,ymin,ymax = (f[k] for k in ('x_min','x_max','y_min','y_max'))
    cx,cy = (xmin+xmax)/2, (ymin+ymax)/2
    width,height = xmax-xmin,ymax-ymin
    specs = {
        'field_floor': ((cx,cy,0.001), (width,height,0.002)),
        'wall_south': ((cx,ymin-0.025,0.25), (width,0.05,0.5)),
        'wall_north': ((cx,ymax+0.025,0.25), (width,0.05,0.5)),
        'wall_west': ((xmin-0.025,cy,0.25), (0.05,height+0.1,0.5)),
        'wall_east': ((xmax+0.025,cy,0.25), (0.05,height+0.1,0.5)),
    }
    d = scene['destination']
    specs['destination_zone'] = (((d['x_min']+d['x_max'])/2,(d['y_min']+d['y_max'])/2,0.003),
                                 (d['x_max']-d['x_min'],d['y_max']-d['y_min'],0.002))
    start = cfg['start_pose']
    specs['start_zone'] = ((start['x'],start['y'],0.003),(0.8,0.6,0.002))
    for name, (p, size) in specs.items():
        model = models[name]
        model.find('pose').text = ' '.join(map(str, (*p,0,0,0)))
        for node in model.findall('.//box/size'):
            node.text = ' '.join(map(str,size))
    g = scene['rack']
    for r in cfg['racks'].values():
        model = models[r['model']]
        if not include_racks:
            world.remove(model)
            continue
        p = r['pose']
        model.find('pose').text = f'{p["x"]} {p["y"]} 0 0 0 {p["yaw"]}'
        for side, sign in (('west',-1),('east',1)):
            link = model.find(f"link[@name='{side}_wall']")
            link.find('pose').text = f'{sign*(g["gap"]+g["wall_thickness"])/2} 0 {g["height"]/2} 0 0 0'
            for node in link.findall('.//box/size'):
                node.text = f'{g["wall_thickness"]} {g["depth"]} {g["height"]}'
        link = model.find("link[@name='qr_panel']")
        values = link.find('pose').text.split()
        values[2] = str(g['qr_height'])
        link.find('pose').text = ' '.join(values)
    for key,m in scene['markers'].items():
        model = models[f'marker_{key.lower()}']
        model.find('pose').text = f'{m["x"]} {m["y"]} 0 0 0 0'
        for node in model.findall('.//cylinder/radius'):
            node.text = str(m['radius'])
    ET.indent(root)
    ET.ElementTree(root).write(target, encoding='utf-8', xml_declaration=True)
    return str(target)
