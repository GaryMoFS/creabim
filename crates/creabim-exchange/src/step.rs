//! Minimal STEP AP214 (ISO-10303-21) B-rep exporter for selected CreaBIM
//! elements. Merges coplanar triangles into analytical `PLANE` faces and
//! straight `LINE` boundary loops so CAD tools like SolidCraft import roofs,
//! slabs, and walls as native B-rep faces and selectable 3D edges.

use creabim_core::{ElementId, Model};
use glam::DVec3;
use std::collections::{HashMap, HashSet};
use std::fmt::Write;

struct StepWriter {
    next_id: usize,
    out: String,
}

impl StepWriter {
    fn alloc(&mut self, line: &str) -> usize {
        let id = self.next_id;
        self.next_id += 1;
        let _ = writeln!(self.out, "#{id}={line};");
        id
    }

    fn vert(
        &mut self,
        vert_map: &mut HashMap<[i64; 3], (usize, usize)>,
        p: DVec3,
    ) -> (usize, usize) {
        let mm = p * 1000.0;
        let key = [
            (mm.x * 100.0).round() as i64,
            (mm.y * 100.0).round() as i64,
            (mm.z * 100.0).round() as i64,
        ];
        if let Some(&ids) = vert_map.get(&key) {
            return ids;
        }
        let cp = self.alloc(&format!(
            "CARTESIAN_POINT('',({:.4},{:.4},{:.4}))",
            mm.x, mm.y, mm.z
        ));
        let vp = self.alloc(&format!("VERTEX_POINT('',#{cp})"));
        vert_map.insert(key, (cp, vp));
        (cp, vp)
    }
}

/// Export the specified elements (or all drawn elements if `ids` is empty)
/// as a STEP AP214 string in millimetres, preserving CreaBIM Z-up world
/// coordinates.
pub fn export_step_subset(model: &Model, ids: &[ElementId]) -> String {
    let want: HashSet<ElementId> = ids.iter().copied().collect();
    let scene: Vec<_> = creabim_geom::scene::drawn_elements(model)
        .into_iter()
        .filter(|item| want.is_empty() || want.contains(&item.id))
        .collect();

    let mut w = StepWriter {
        next_id: 30,
        out: String::from(
            "ISO-10303-21;\n\
             HEADER;\n\
             FILE_DESCRIPTION(('CreaBIM B-rep export'),'2;1');\n\
             FILE_NAME('context.step','2026-01-01T00:00:00',(''),(''),'CreaBIM','','');\n\
             FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }'));\n\
             ENDSEC;\n\
             DATA;\n\
             #10=APPLICATION_CONTEXT('automotive design');\n\
             #11=APPLICATION_PROTOCOL_DEFINITION('international standard','automotive_design',2000,#10);\n\
             #12=PRODUCT_CONTEXT('',#10,'mechanical');\n\
             #13=PRODUCT('CreaBIM','CreaBIM','',(#12));\n\
             #14=PRODUCT_DEFINITION_FORMATION('','',#13);\n\
             #15=PRODUCT_DEFINITION_CONTEXT('part definition',#10,'design');\n\
             #16=PRODUCT_DEFINITION('design','',#14,#15);\n\
             #17=PRODUCT_DEFINITION_SHAPE('','',#16);\n\
             #18=(GEOMETRIC_REPRESENTATION_CONTEXT(3) GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#22)) GLOBAL_UNIT_ASSIGNED_CONTEXT((#19,#20,#21)) REPRESENTATION_CONTEXT('Context3D','3D Context'));\n\
             #19=(LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.));\n\
             #20=(NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.));\n\
             #21=(NAMED_UNIT(*) SI_UNIT($,.STERADIAN.) SOLID_ANGLE_UNIT());\n\
             #22=UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(0.01),#19,'distance_accuracy_value','confusion accuracy');\n\
             #23=CARTESIAN_POINT('',(0.,0.,0.));\n\
             #24=DIRECTION('',(0.,0.,1.));\n\
             #25=DIRECTION('',(1.,0.,0.));\n\
             #26=AXIS2_PLACEMENT_3D('',#23,#24,#25);\n",
        ),
    };

    let mut shape_items = vec![26usize];

    for item in &scene {
        let faces = extract_planar_faces(&item.mesh.positions, &item.mesh.indices);
        if faces.is_empty() {
            continue;
        }
        let mut vert_map: HashMap<[i64; 3], (usize, usize)> = HashMap::new();
        let mut step_faces = Vec::new();
        for face in faces {
            let orig_mm = face.origin * 1000.0;
            let orig = w.alloc(&format!(
                "CARTESIAN_POINT('',({:.4},{:.4},{:.4}))",
                orig_mm.x, orig_mm.y, orig_mm.z
            ));
            let n_dir = w.alloc(&format!(
                "DIRECTION('',({:.6},{:.6},{:.6}))",
                face.normal.x, face.normal.y, face.normal.z
            ));
            let ref_axis = if face.normal.z.abs() < 0.9 {
                DVec3::Z.cross(face.normal).normalize()
            } else {
                DVec3::X.cross(face.normal).normalize()
            };
            let x_dir = w.alloc(&format!(
                "DIRECTION('',({:.6},{:.6},{:.6}))",
                ref_axis.x, ref_axis.y, ref_axis.z
            ));
            let ax = w.alloc(&format!("AXIS2_PLACEMENT_3D('',#{orig},#{n_dir},#{x_dir})"));
            let plane = w.alloc(&format!("PLANE('',#{ax})"));

            let mut bounds = Vec::new();
            for (li, loop_pts) in face.loops.iter().enumerate() {
                if loop_pts.len() < 3 {
                    continue;
                }
                let verts: Vec<(usize, usize)> = loop_pts
                    .iter()
                    .map(|&p| w.vert(&mut vert_map, p))
                    .collect();
                let mut oriented_edges = Vec::new();
                for i in 0..verts.len() {
                    let j = (i + 1) % verts.len();
                    let p0 = loop_pts[i];
                    let p1 = loop_pts[j];
                    let delta = (p1 - p0) * 1000.0;
                    let len = delta.length();
                    if len < 1e-4 {
                        continue;
                    }
                    let d = delta / len;
                    let dir_id = w.alloc(&format!(
                        "DIRECTION('',({:.6},{:.6},{:.6}))",
                        d.x, d.y, d.z
                    ));
                    let vec_id = w.alloc(&format!("VECTOR('',#{dir_id},{len:.4})"));
                    let line_id = w.alloc(&format!("LINE('',#{},#{vec_id})", verts[i].0));
                    let ec = w.alloc(&format!(
                        "EDGE_CURVE('',#{},#{},#{line_id},.T.)",
                        verts[i].1, verts[j].1
                    ));
                    let oe = w.alloc(&format!("ORIENTED_EDGE('',*,*,#{ec},.T.)"));
                    oriented_edges.push(oe);
                }
                if oriented_edges.len() < 3 {
                    continue;
                }
                let list_str = oriented_edges
                    .iter()
                    .map(|e| format!("#{e}"))
                    .collect::<Vec<_>>()
                    .join(",");
                let el = w.alloc(&format!("EDGE_LOOP('',({list_str}))"));
                let bound_kind = if li == 0 { "FACE_OUTER_BOUND" } else { "FACE_BOUND" };
                bounds.push(w.alloc(&format!("{bound_kind}('',#{el},.T.)")));
            }
            if bounds.is_empty() {
                continue;
            }
            let b_str = bounds
                .iter()
                .map(|b| format!("#{b}"))
                .collect::<Vec<_>>()
                .join(",");
            step_faces.push(w.alloc(&format!("ADVANCED_FACE('',({b_str}),#{plane},.T.)")));
        }

        if !step_faces.is_empty() {
            let f_str = step_faces
                .iter()
                .map(|f| format!("#{f}"))
                .collect::<Vec<_>>()
                .join(",");
            let shell = w.alloc(&format!("OPEN_SHELL('',({f_str}))"));
            let sbsm = w.alloc(&format!("SHELL_BASED_SURFACE_MODEL('',(#{shell}))"));
            shape_items.push(sbsm);
        }
    }

    let items_str = shape_items
        .iter()
        .map(|i| format!("#{i}"))
        .collect::<Vec<_>>()
        .join(",");
    let rep = w.alloc(&format!(
        "MANIFOLD_SURFACE_SHAPE_REPRESENTATION('',({items_str}),#18)"
    ));
    let _ = w.alloc(&format!("SHAPE_DEFINITION_REPRESENTATION(#17,#{rep})"));
    w.out.push_str("ENDSEC;\nEND-ISO-10303-21;\n");
    w.out
}

struct PlanarFace {
    origin: DVec3,
    normal: DVec3,
    loops: Vec<Vec<DVec3>>,
}

type PtKey = [i64; 3];

fn pt_key(p: DVec3) -> PtKey {
    [
        (p.x * 1e5).round() as i64,
        (p.y * 1e5).round() as i64,
        (p.z * 1e5).round() as i64,
    ]
}

fn extract_planar_faces(positions: &[[f32; 3]], indices: &[u32]) -> Vec<PlanarFace> {
    // Group coplanar triangles by quantized (normal, plane_d)
    let mut groups: HashMap<[i64; 4], Vec<[DVec3; 3]>> = HashMap::new();
    for tri in indices.chunks_exact(3) {
        let p0 = DVec3::from_array(positions[tri[0] as usize].map(f64::from));
        let p1 = DVec3::from_array(positions[tri[1] as usize].map(f64::from));
        let p2 = DVec3::from_array(positions[tri[2] as usize].map(f64::from));
        let n = (p1 - p0).cross(p2 - p0);
        let len = n.length();
        if len < 1e-10 {
            continue;
        }
        let n = n / len;
        let d = n.dot(p0);
        let key = [
            (n.x * 1e4).round() as i64,
            (n.y * 1e4).round() as i64,
            (n.z * 1e4).round() as i64,
            (d * 1e4).round() as i64,
        ];
        groups.entry(key).or_default().push([p0, p1, p2]);
    }

    let mut out = Vec::new();
    for (_key, tris) in groups {
        let n = (tris[0][1] - tris[0][0])
            .cross(tris[0][2] - tris[0][0])
            .normalize_or_zero();
        if n == DVec3::ZERO {
            continue;
        }
        let origin = tris[0][0];

        // Directed half-edges: an edge (a, b) whose opposite (b, a) is absent is on the boundary
        let mut dir_edges: HashMap<(PtKey, PtKey), (DVec3, DVec3)> = HashMap::new();
        for [a, b, c] in &tris {
            for (p, q) in [(*a, *b), (*b, *c), (*c, *a)] {
                let kp = pt_key(p);
                let kq = pt_key(q);
                if kp == kq {
                    continue;
                }
                if dir_edges.remove(&(kq, kp)).is_none() {
                    dir_edges.insert((kp, kq), (p, q));
                }
            }
        }
        if dir_edges.is_empty() {
            continue;
        }

        // Chain boundary edges into closed loops
        let mut adj: HashMap<PtKey, Vec<(PtKey, DVec3, DVec3)>> = HashMap::new();
        for ((ka, kb), (pa, pb)) in dir_edges {
            adj.entry(ka).or_default().push((kb, pa, pb));
        }
        let mut loops = Vec::new();
        while let Some((&start, _)) = adj.iter().find(|(_, v)| !v.is_empty()) {
            let mut raw = Vec::new();
            let mut cur = start;
            loop {
                let Some(nexts) = adj.get_mut(&cur) else {
                    break;
                };
                if nexts.is_empty() {
                    break;
                }
                let (nxt, pa, _pb) = nexts.pop().unwrap();
                raw.push(pa);
                cur = nxt;
                if cur == start {
                    break;
                }
            }
            if raw.len() >= 3 {
                let simplified = simplify_collinear(&raw);
                if simplified.len() >= 3 {
                    loops.push(simplified);
                }
            }
        }
        if loops.is_empty() {
            continue;
        }
        // Largest signed area loop first (outer boundary)
        loops.sort_by(|a, b| {
            let area = |pts: &[DVec3]| -> f64 {
                let mut sum = DVec3::ZERO;
                for i in 0..pts.len() {
                    let j = (i + 1) % pts.len();
                    sum += (pts[i] - origin).cross(pts[j] - origin);
                }
                sum.dot(n).abs()
            };
            area(b).partial_cmp(&area(a)).unwrap_or(std::cmp::Ordering::Equal)
        });
        out.push(PlanarFace {
            origin,
            normal: n,
            loops,
        });
    }
    out
}

fn simplify_collinear(pts: &[DVec3]) -> Vec<DVec3> {
    let n = pts.len();
    let mut kept = Vec::with_capacity(n);
    for i in 0..n {
        let prev = pts[(i + n - 1) % n];
        let curr = pts[i];
        let next = pts[(i + 1) % n];
        let d1 = (curr - prev).normalize_or_zero();
        let d2 = (next - curr).normalize_or_zero();
        if d1.dot(d2) < 0.9999 {
            kept.push(curr);
        }
    }
    kept
}
