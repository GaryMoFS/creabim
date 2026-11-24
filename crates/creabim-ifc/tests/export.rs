use creabim_core::sample::sample_house;
use creabim_ifc::{export_ifc4, ExportOptions};

#[test]
fn sample_house_exports_a_complete_ifc4_file() {
    let model = sample_house();
    let ifc = export_ifc4(&model, &ExportOptions::default());
    assert!(ifc.starts_with("ISO-10303-21;"));
    assert!(ifc.contains("FILE_SCHEMA(('IFC4'));"));
    assert!(ifc.trim_end().ends_with("END-ISO-10303-21;"));

    let count = |entity: &str| ifc.matches(&format!("={entity}(")).count();
    assert_eq!(count("IFCPROJECT"), 1);
    assert_eq!(count("IFCBUILDINGSTOREY"), 2);
    assert_eq!(count("IFCWALL"), 9);
    assert_eq!(count("IFCSLAB"), 2);
    assert_eq!(count("IFCDOOR"), 2);
    assert_eq!(count("IFCWINDOW"), 11);
    assert_eq!(count("IFCOPENINGELEMENT"), 13);
    assert_eq!(count("IFCRELVOIDSELEMENT"), 13);
    assert_eq!(count("IFCRELFILLSELEMENT"), 13);
    assert_eq!(count("IFCROOF"), 1);
    assert_eq!(count("IFCMATERIAL"), 4);
    assert_eq!(count("IFCRELASSOCIATESMATERIAL"), 4);
    assert_eq!(count("IFCWINDOWLININGPROPERTIES"), 11);
    assert_eq!(count("IFCWINDOWPANELPROPERTIES"), 11);
    assert_eq!(count("IFCDOORLININGPROPERTIES"), 2);
    assert_eq!(count("IFCDOORPANELPROPERTIES"), 2);
    assert!(ifc.contains("Pset_DoorCommon"));
    assert!(ifc.contains("Pset_SlabCommon"));
    assert!(ifc.contains(".DOUBLE_PANEL_VERTICAL."));
    assert!(ifc.contains("IFCSTYLEDITEM(#"));

    // Element GlobalIds are the model's own ids, so re-exports keep identity.
    for e in model.iter().filter(|e| e.ifc_class() == "IfcWall") {
        assert!(ifc.contains(&format!("'{}'", e.id.ifc_guid())));
    }
}

#[test]
fn windows_export_glass_frame_and_reveal_properties() {
    use creabim_core::{library::LIBRARY, Document, ElementKind, OpeningKind, Transaction};

    let model = sample_house();
    let timber_id = model.iter().find(|e| e.name == "Timber").unwrap().id;
    let glass = LIBRARY
        .iter()
        .find(|e| e.name == "Glass, clear")
        .unwrap()
        .element();
    let win = model
        .iter()
        .find(|e| matches!(&e.kind, ElementKind::Opening(o) if o.kind == OpeningKind::Window))
        .unwrap()
        .clone();
    let mut updated = win.clone();
    if let ElementKind::Opening(o) = &mut updated.kind {
        o.material = Some(timber_id);
        o.frame.depth = 0.09;
        o.frame.reveal = Some(0.05);
    }
    let mut doc = Document::from_model(model);
    doc.commit(
        Transaction::new("reveal")
            .insert(glass)
            .replace(win, updated),
    )
    .unwrap();
    let ifc = export_ifc4(doc.model(), &ExportOptions::default());

    assert!(ifc.contains("IFCMATERIALCONSTITUENT('Frame',$,#"));
    assert!(ifc.contains("IFCMATERIALCONSTITUENT('Glazing',$,#"));
    assert!(ifc.contains("IFCWINDOWLININGPROPERTIES("));
    assert!(ifc.contains(",0.09,0.06,$,0.036,$,$,0.5,$,$,0.05,0.06,0.035);"));
    assert!(ifc.contains("IFCWINDOWPANELPROPERTIES("));
    assert!(ifc.contains(".DOUBLE_PANEL_VERTICAL."));
    // Both frame solids and glass solids get per-item surface styles.
    assert!(ifc.matches("IFCSTYLEDITEM(#").count() >= 8);
}

#[test]
fn walls_slabs_and_roofs_carry_styled_items() {
    let model = creabim_core::starter::starter_house();
    let ifc = export_ifc4(&model, &ExportOptions::default());

    let lines: std::collections::HashMap<String, &str> = ifc
        .lines()
        .filter_map(|l| {
            let l = l.trim();
            l.starts_with('#').then(|| l.split_once('='))?
        })
        .map(|(id, rest)| (id.to_string(), rest))
        .collect();

    let styled_targets: std::collections::HashSet<String> = ifc
        .lines()
        .filter_map(|l| {
            let rest = l.trim().split_once('=')?.1.trim_start();
            let args = rest.strip_prefix("IFCSTYLEDITEM(")?;
            Some(args.split(',').next()?.trim().to_string())
        })
        .collect();

    let first_ref = |s: &str| -> Option<String> {
        let start = s.find('#')?;
        let tail = &s[start..];
        let end = tail[1..].find(|c: char| !c.is_ascii_digit()).map_or(tail.len(), |i| i + 1);
        Some(tail[..end].to_string())
    };

    for kind in ["IFCWALL(", "IFCSLAB(", "IFCROOF("] {
        let mut checked = 0usize;
        for line in ifc.lines() {
            let Some((_, rest)) = line.trim().split_once('=') else {
                continue;
            };
            if !rest.trim_start().starts_with(kind) {
                continue;
            }
            // Product arg 7 (0-indexed 6) is Representation (#IFCPRODUCTDEFINITIONSHAPE)
            let args: Vec<&str> = rest.split(',').collect();
            let shape_id = first_ref(args[6]).expect("product representation");
            let shape_line = lines[&shape_id];
            // Shape arg 3 is list of representations: (#rep1,...)
            let reps_start = shape_line.find('(').unwrap();
            let body_rep_id = first_ref(&shape_line[reps_start..]).expect("body rep");
            let body_rep_line = lines[&body_rep_id];
            // IFCSHAPEREPRESENTATION(..., (#item1,...))
            let items_start = body_rep_line.rfind('(').unwrap();
            let item_id = first_ref(&body_rep_line[items_start..]).expect("body geom item");
            assert!(
                styled_targets.contains(&item_id),
                "{kind} geometry item {item_id} missing IFCSTYLEDITEM"
            );
            checked += 1;
        }
        assert!(checked > 0, "expected at least one {kind}");
    }

    for wall_name in ["'GF Wing South'", "'GF Wing East'"] {
        let wall_line = ifc
            .lines()
            .find(|l| l.contains("IFCWALL(") && l.contains(wall_name))
            .expect("wing wall in starter house");
        let rest = wall_line.split_once('=').unwrap().1;
        let shape_id = first_ref(rest.split(',').collect::<Vec<_>>()[6]).unwrap();
        let shape_line = lines[&shape_id];
        let body_rep_id = first_ref(&shape_line[shape_line.find('(').unwrap()..]).unwrap();
        assert!(
            lines[&body_rep_id].contains("'SweptSolid'"),
            "{wall_name} should export as SweptSolid when flat roof meets wall top: {}",
            lines[&body_rep_id]
        );
    }
}
