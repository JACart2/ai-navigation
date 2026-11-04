#!/usr/bin/env python3
"""Generate an offline interactive keyframe/timestamp reference for overlap selection."""
import argparse
import html
import json
from pathlib import Path
import subprocess
import tempfile

from pipeline import read_frames


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('simplemap', type=Path)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    with tempfile.TemporaryDirectory(prefix='mola-overlap-') as folder:
        tum = Path(folder)/'frames.tum'
        subprocess.run(['sm-cli', 'export-keyframes', str(a.simplemap), '--output', str(tum)], check=True)
        frames = read_frames(tum)
    data = json.dumps(frames[:, :4].tolist(), allow_nan=False)
    title = html.escape(a.simplemap.name)
    content = '''<!doctype html><html lang="en"><meta charset="utf-8">
<title>Offline overlap review</title>
<style>body{font:16px system-ui;max-width:1100px;margin:2rem auto;padding:1rem}svg{width:100%;height:60vh;background:#f4f6f8}input{width:100%}button{margin:.4rem}pre{white-space:pre-wrap}circle{stroke:white;stroke-width:2}</style>
<h1>Offline overlap review: TITLE</h1>
<p>Select a keyframe on the trajectory or use the slider. Mark the entry and exit of each overlap traversal. IDs refer to the saved offline SimpleMap.</p>
<label>Projection <select id="projection"><option value="2">X–Y</option><option value="3">X–Z</option></select></label>
<svg id="plot" viewBox="0 0 1000 600" aria-label="Offline keyframe trajectory"></svg>
<input id="frame" type="range" min="0" step="1" value="0"><p id="details"></p>
<button data-mark="start_enter">First traversal: enter</button><button data-mark="start_leave">First traversal: leave</button>
<button data-mark="end_enter">Final traversal: enter</button><button data-mark="end_leave">Final traversal: leave</button><pre id="ranges"></pre>
<p>This view shows estimated keyframe poses. Confirm recognizable shared structure in the scans. To inspect those scans separately:</p>
<pre>source /root/dev_ws/install/setup.bash
sm-cli export-rawlog INPUT_PATH --output NEW_REVIEW.rawlog
rawlog-viewer NEW_REVIEW.rawlog</pre>
<script>
const frames=FRAME_DATA, marks={}, slider=document.getElementById('frame'), svg=document.getElementById('plot');let coords=[];
slider.max=frames.length-1;
function draw(){const axis=Number(document.getElementById('projection').value),xs=frames.map(p=>p[1]),ys=frames.map(p=>p[axis]);
const minx=Math.min(...xs),maxx=Math.max(...xs),miny=Math.min(...ys),maxy=Math.max(...ys),scale=Math.min(940/Math.max(1,maxx-minx),540/Math.max(1,maxy-miny));
coords=frames.map(p=>[30+(p[1]-minx)*scale,570-(p[axis]-miny)*scale]);
svg.innerHTML='<polyline fill="none" stroke="#476d9c" stroke-width="2" points="'+coords.map(p=>p.join(',')).join(' ')+'"/><circle id="cursor" r="7" fill="#d43f3a"/>';update();}
function update(){const i=Number(slider.value),p=frames[i],c=document.getElementById('cursor');c.setAttribute('cx',coords[i][0]);c.setAttribute('cy',coords[i][1]);document.getElementById('details').textContent=`Keyframe ${i} | ROS timestamp ${p[0].toFixed(6)} | XYZ ${p.slice(1).map(v=>v.toFixed(2)).join(', ')} m`;}
slider.oninput=update;document.getElementById('projection').onchange=draw;
svg.onclick=e=>{const p=svg.createSVGPoint();p.x=e.clientX;p.y=e.clientY;const q=p.matrixTransform(svg.getScreenCTM().inverse());let best=0,d=Infinity;coords.forEach((p,i)=>{const n=(p[0]-q.x)**2+(p[1]-q.y)**2;if(n<d){best=i;d=n;}});slider.value=best;update();};
document.querySelectorAll('button').forEach(b=>b.onclick=()=>{marks[b.dataset.mark]=Number(slider.value);let text=JSON.stringify(marks,null,2);if(Object.keys(marks).length===4){text+='\\n--loop-start-keyframes '+marks.start_enter+':'+marks.start_leave+' --loop-end-keyframes '+marks.end_enter+':'+marks.end_leave;text+='\\n--loop-start-times '+frames[marks.start_enter][0].toFixed(6)+':'+frames[marks.start_leave][0].toFixed(6)+' --loop-end-times '+frames[marks.end_enter][0].toFixed(6)+':'+frames[marks.end_leave][0].toFixed(6);}document.getElementById('ranges').textContent=text;});draw();
</script></html>'''
    # JSON contains only finite numeric arrays; paths are HTML-escaped and never JavaScript.
    content = content.replace('TITLE', title).replace('INPUT_PATH', html.escape(str(a.simplemap.resolve())))
    content = content.replace('FRAME_DATA', data)
    if a.output.exists():
        raise FileExistsError(f'Review output already exists: {a.output}')
    a.output.write_text(content)
    print(a.output)


if __name__ == '__main__':
    main()
