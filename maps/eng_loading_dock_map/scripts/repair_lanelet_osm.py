#!/usr/bin/env python3
"""Minimal Lanelet2/OSM integrity repair for Autoware map loader.

Does not redraw geometry. Removes/cleans only broken primitives that prevent load.
Writes finalMap_repaired.osm (never overwrites finalMap.osm).
"""

from __future__ import annotations

import json
import re
import xml.etree.ElementTree as ET
from collections import defaultdict
from copy import deepcopy
from pathlib import Path

MAP_DIR = Path("/home/agxorin/autoware_data/maps/eng_loading_dock_map")
SRC = MAP_DIR / "finalMap.osm"
OUT = MAP_DIR / "finalMap_repaired.osm"
REPORT_DIR = MAP_DIR / "intermediate" / "lanelet_repair"
REPORT_DIR.mkdir(parents=True, exist_ok=True)


def parse_osm(path: Path) -> ET.ElementTree:
    # Strip XML namespaces if any for simpler handling
    raw = path.read_bytes()
    # ElementTree
    return ET.parse(path)


def index_map(root: ET.Element):
    nodes = {}
    ways = {}
    relations = {}
    for el in list(root):
        tag = el.tag.split("}")[-1]
        if tag == "node":
            nodes[int(el.attrib["id"])] = el
        elif tag == "way":
            ways[int(el.attrib["id"])] = el
        elif tag == "relation":
            relations[int(el.attrib["id"])] = el
    return nodes, ways, relations


def way_nd_refs(way: ET.Element) -> list[int]:
    return [int(nd.attrib["ref"]) for nd in way if nd.tag.split("}")[-1] == "nd"]


def way_tags(way: ET.Element) -> dict[str, str]:
    return {
        t.attrib["k"]: t.attrib["v"]
        for t in way
        if t.tag.split("}")[-1] == "tag" and "k" in t.attrib
    }


def rel_tags(rel: ET.Element) -> dict[str, str]:
    return {
        t.attrib["k"]: t.attrib["v"]
        for t in rel
        if t.tag.split("}")[-1] == "tag" and "k" in t.attrib
    }


def rel_members(rel: ET.Element) -> list[tuple[str, int, str]]:
    out = []
    for m in rel:
        if m.tag.split("}")[-1] != "member":
            continue
        out.append((m.attrib["type"], int(m.attrib["ref"]), m.attrib.get("role", "")))
    return out


def validate(nodes, ways, relations) -> list[dict]:
    errors = []
    node_ids = set(nodes)

    # duplicate IDs across types
    for i in set(nodes) & set(ways):
        errors.append(
            {
                "category": "duplicate_ids",
                "id": i,
                "msg": f"ID {i} used as both node and way",
            }
        )
    for i in set(ways) & set(relations):
        errors.append(
            {
                "category": "duplicate_ids",
                "id": i,
                "msg": f"ID {i} used as both way and relation",
            }
        )
    for i in set(nodes) & set(relations):
        errors.append(
            {
                "category": "duplicate_ids",
                "id": i,
                "msg": f"ID {i} used as both node and relation",
            }
        )

    broken_ways = set()
    for wid, way in ways.items():
        nds = way_nd_refs(way)
        missing = [n for n in nds if n not in node_ids]
        tags = way_tags(way)
        if missing:
            errors.append(
                {
                    "category": "missing_references",
                    "id": wid,
                    "kind": "way",
                    "msg": f"Way {wid} references nonexisting points {missing}",
                    "missing_nodes": missing,
                    "tags": tags,
                }
            )
            broken_ways.add(wid)
        if len([n for n in nds if n in node_ids]) == 0:
            errors.append(
                {
                    "category": "broken_way_geometry",
                    "id": wid,
                    "kind": "way",
                    "msg": f"Way {wid}: Ways must have at least one point!",
                    "tags": tags,
                }
            )
            broken_ways.add(wid)

    # relations referencing missing primitives
    for rid, rel in relations.items():
        tags = rel_tags(rel)
        for typ, ref, role in rel_members(rel):
            if typ == "node" and ref not in nodes:
                errors.append(
                    {
                        "category": "missing_references",
                        "id": rid,
                        "kind": "relation",
                        "msg": f"Relation {rid} member node {ref} role={role} missing",
                    }
                )
            if typ == "way" and ref not in ways:
                errors.append(
                    {
                        "category": "missing_references",
                        "id": rid,
                        "kind": "relation",
                        "msg": f"Relation {rid} member way {ref} role={role} missing",
                    }
                )
            if typ == "relation" and ref not in relations:
                errors.append(
                    {
                        "category": "missing_references",
                        "id": rid,
                        "kind": "relation",
                        "msg": f"Relation {rid} member relation {ref} role={role} missing",
                    }
                )

    # lanelets
    for rid, rel in relations.items():
        tags = rel_tags(rel)
        if tags.get("type") != "lanelet":
            continue
        members = rel_members(rel)
        left = [m for m in members if m[2] == "left"]
        right = [m for m in members if m[2] == "right"]
        if len(left) != 1:
            errors.append(
                {
                    "category": "disconnected_lanelets",
                    "id": rid,
                    "kind": "lanelet",
                    "msg": f"Lanelet {rid} has not exactly one left border! (count={len(left)})",
                    "members": members,
                    "tags": tags,
                }
            )
        if len(right) != 1:
            errors.append(
                {
                    "category": "disconnected_lanelets",
                    "id": rid,
                    "kind": "lanelet",
                    "msg": f"Lanelet {rid} has not exactly one right border! (count={len(right)})",
                    "members": members,
                    "tags": tags,
                }
            )
        for typ, ref, role in left + right:
            if typ != "way" or ref in broken_ways or ref not in ways:
                errors.append(
                    {
                        "category": "broken_way_geometry",
                        "id": rid,
                        "kind": "lanelet",
                        "msg": f"Lanelet {rid} {role} border way {ref} is missing/broken",
                        "tags": tags,
                    }
                )
            elif len(way_nd_refs(ways[ref])) < 2:
                # borders typically need >=2 points for a linestring
                errors.append(
                    {
                        "category": "broken_way_geometry",
                        "id": rid,
                        "kind": "lanelet",
                        "msg": f"Lanelet {rid} {role} border way {ref} has <2 points",
                        "tags": tags,
                    }
                )

    # traffic light regulatory elements
    for rid, rel in relations.items():
        tags = rel_tags(rel)
        if tags.get("type") != "regulatory_element":
            continue
        if tags.get("subtype") != "traffic_light":
            continue
        members = rel_members(rel)
        refers = [m for m in members if m[2] == "refers"]
        ok_refers = False
        for typ, ref, role in refers:
            if typ == "way" and ref in ways and len(way_nd_refs(ways[ref])) >= 1:
                # and ideally type=traffic_light
                ok_refers = True
        if not ok_refers:
            errors.append(
                {
                    "category": "traffic_light_inconsistencies",
                    "id": rid,
                    "kind": "regulatory_element",
                    "msg": (
                        f"RegulatoryElement {rid}: Creating a regulatory element of type "
                        f"traffic_light failed: No traffic light defined!"
                    ),
                    "members": members,
                    "tags": tags,
                }
            )

    # other regulatory elements with missing member targets
    for rid, rel in relations.items():
        tags = rel_tags(rel)
        if tags.get("type") != "regulatory_element":
            continue
        for typ, ref, role in rel_members(rel):
            if typ == "way" and ref in broken_ways:
                errors.append(
                    {
                        "category": "invalid_regulatory_elements",
                        "id": rid,
                        "kind": "regulatory_element",
                        "msg": f"RegulatoryElement {rid} references broken way {ref} role={role}",
                        "tags": tags,
                    }
                )

    return errors


def remove_element(parent: ET.Element, el: ET.Element) -> None:
    parent.remove(el)


def repair(root: ET.Element) -> dict:
    actions = []
    nodes, ways, relations = index_map(root)

    # 1) Strip missing nd refs from ways; delete ways with zero remaining points
    for wid in list(ways.keys()):
        way = ways[wid]
        nds = list(way)
        changed = False
        for nd in list(nds):
            if nd.tag.split("}")[-1] != "nd":
                continue
            ref = int(nd.attrib["ref"])
            if ref not in nodes:
                way.remove(nd)
                changed = True
                actions.append(
                    {
                        "action": "remove_missing_nd_from_way",
                        "way_id": wid,
                        "node_id": ref,
                    }
                )
        if len(way_nd_refs(way)) == 0:
            tags = way_tags(way)
            remove_element(root, way)
            del ways[wid]
            actions.append({"action": "delete_empty_way", "way_id": wid, "tags": tags})

    # Re-index after way deletions
    nodes, ways, relations = index_map(root)

    # Delete remaining empty ways (safety)
    for wid in list(ways.keys()):
        if len(way_nd_refs(ways[wid])) == 0:
            tags = way_tags(ways[wid])
            remove_element(root, ways[wid])
            del ways[wid]
            actions.append({"action": "delete_empty_way", "way_id": wid, "tags": tags})

    nodes, ways, relations = index_map(root)

    # 2) Delete traffic_light regulatory elements without valid refers
    bad_tl_regs = set()
    for rid, rel in list(relations.items()):
        tags = rel_tags(rel)
        if tags.get("type") != "regulatory_element" or tags.get("subtype") != "traffic_light":
            continue
        refers = [m for m in rel_members(rel) if m[2] == "refers"]
        ok = any(
            typ == "way" and ref in ways and len(way_nd_refs(ways[ref])) >= 1
            for typ, ref, _ in refers
        )
        if not ok:
            bad_tl_regs.add(rid)
            remove_element(root, rel)
            del relations[rid]
            actions.append(
                {
                    "action": "delete_invalid_traffic_light_regulatory_element",
                    "relation_id": rid,
                    "tags": tags,
                    "members": refers,
                }
            )

    nodes, ways, relations = index_map(root)

    # 3) Strip relation members pointing at deleted ways/relations
    for rid, rel in list(relations.items()):
        removed_members = []
        for m in list(rel):
            if m.tag.split("}")[-1] != "member":
                continue
            typ = m.attrib["type"]
            ref = int(m.attrib["ref"])
            role = m.attrib.get("role", "")
            missing = (
                (typ == "way" and ref not in ways)
                or (typ == "relation" and ref not in relations)
                or (typ == "node" and ref not in nodes)
            )
            if missing:
                rel.remove(m)
                removed_members.append((typ, ref, role))
        if removed_members:
            actions.append(
                {
                    "action": "strip_missing_members",
                    "relation_id": rid,
                    "removed": removed_members,
                    "tags": rel_tags(rel),
                }
            )

    nodes, ways, relations = index_map(root)

    # 4) Delete lanelets that still lack exactly one valid left and right border
    for rid, rel in list(relations.items()):
        tags = rel_tags(rel)
        if tags.get("type") != "lanelet":
            continue
        members = rel_members(rel)
        left = [m for m in members if m[2] == "left"]
        right = [m for m in members if m[2] == "right"]

        def border_ok(m):
            typ, ref, role = m
            return (
                typ == "way"
                and ref in ways
                and len(way_nd_refs(ways[ref])) >= 1
            )

        ok = len(left) == 1 and len(right) == 1 and border_ok(left[0]) and border_ok(right[0])
        if not ok:
            remove_element(root, rel)
            del relations[rid]
            actions.append(
                {
                    "action": "delete_broken_lanelet",
                    "relation_id": rid,
                    "tags": tags,
                    "members": members,
                    "reason": f"left={left} right={right}",
                }
            )

    nodes, ways, relations = index_map(root)

    # 5) Delete other relations that reference missing primitives critically
    #    (e.g. regulatory_element left dangling) — already stripped members;
    #    remove empty invalid traffic lights already done.

    # 6) Optionally delete orphan empty traffic_light ways already deleted in step 1

    # 7) If a highway way still has only 1 point after nd strip, delete it (invalid linestring)
    for wid in list(ways.keys()):
        n = len(way_nd_refs(ways[wid]))
        if n < 2:
            # Keep single-point only if somehow needed; Lanelet2 linestrings need 2+
            # Autoware error for empty was "at least one point"; for borders typically 2+
            tags = way_tags(ways[wid])
            # Delete ways with 0 already handled; with 1 point delete to be safe for borders
            if n < 1 or (n == 1):
                # Check if referenced as lanelet border
                used_as_border = False
                for rid, rel in relations.items():
                    if rel_tags(rel).get("type") != "lanelet":
                        continue
                    for typ, ref, role in rel_members(rel):
                        if typ == "way" and ref == wid and role in ("left", "right", "centerline"):
                            used_as_border = True
                if n < 2:
                    # remove from any relations first
                    for rid, rel in list(relations.items()):
                        for m in list(rel):
                            if m.tag.split("}")[-1] != "member":
                                continue
                            if m.attrib["type"] == "way" and int(m.attrib["ref"]) == wid:
                                rel.remove(m)
                    remove_element(root, ways[wid])
                    del ways[wid]
                    actions.append(
                        {
                            "action": "delete_degenerate_way_lt_2_points",
                            "way_id": wid,
                            "n_points": n,
                            "tags": tags,
                            "was_border": used_as_border,
                        }
                    )

    # Re-run lanelet cleanup after degenerate way deletion
    nodes, ways, relations = index_map(root)
    for rid, rel in list(relations.items()):
        tags = rel_tags(rel)
        if tags.get("type") != "lanelet":
            continue
        members = rel_members(rel)
        left = [m for m in members if m[2] == "left"]
        right = [m for m in members if m[2] == "right"]

        def border_ok2(m):
            typ, ref, role = m
            return typ == "way" and ref in ways and len(way_nd_refs(ways[ref])) >= 2

        if not (
            len(left) == 1
            and len(right) == 1
            and border_ok2(left[0])
            and border_ok2(right[0])
        ):
            remove_element(root, rel)
            actions.append(
                {
                    "action": "delete_broken_lanelet_after_way_cleanup",
                    "relation_id": rid,
                    "tags": tags,
                    "members": members,
                }
            )

    return {"actions": actions}


def write_osm(tree: ET.ElementTree, path: Path) -> None:
    root = tree.getroot()
    # Ensure osm attributes preserved
    # Pretty-ish write
    path.parent.mkdir(parents=True, exist_ok=True)
    tree.write(path, encoding="UTF-8", xml_declaration=True)


def main() -> None:
    assert SRC.exists(), SRC
    tree = parse_osm(SRC)
    root = tree.getroot()
    nodes, ways, relations = index_map(root)
    errors_before = validate(nodes, ways, relations)
    (REPORT_DIR / "validation_before.json").write_text(
        json.dumps(errors_before, indent=2, default=str)
    )

    # Categorize counts
    cats = defaultdict(int)
    for e in errors_before:
        cats[e["category"]] += 1
    print("BEFORE errors", len(errors_before), dict(cats))

    # Work on a deep copy via re-parse after writing? ElementTree mutate in place from fresh parse
    tree2 = parse_osm(SRC)
    repair_info = repair(tree2.getroot())
    (REPORT_DIR / "repair_actions.json").write_text(json.dumps(repair_info, indent=2, default=str))

    write_osm(tree2, OUT)
    print("Wrote", OUT)

    # Validate repaired
    tree3 = parse_osm(OUT)
    n3, w3, r3 = index_map(tree3.getroot())
    errors_after = validate(n3, w3, r3)
    (REPORT_DIR / "validation_after.json").write_text(
        json.dumps(errors_after, indent=2, default=str)
    )
    cats2 = defaultdict(int)
    for e in errors_after:
        cats2[e["category"]] += 1
    print("AFTER errors", len(errors_after), dict(cats2))
    if errors_after:
        print("Remaining:")
        for e in errors_after[:50]:
            print(" ", e["category"], e.get("id"), e["msg"])

    summary = {
        "source": str(SRC),
        "output": str(OUT),
        "errors_before": len(errors_before),
        "errors_after": len(errors_after),
        "categories_before": dict(cats),
        "categories_after": dict(cats2),
        "n_actions": len(repair_info["actions"]),
        "action_types": dict(
            defaultdict(
                int,
                **{
                    k: sum(1 for a in repair_info["actions"] if a["action"] == k)
                    for k in {a["action"] for a in repair_info["actions"]}
                },
            )
        ),
    }
    # fix action_types more simply
    at = defaultdict(int)
    for a in repair_info["actions"]:
        at[a["action"]] += 1
    summary["action_types"] = dict(at)
    (REPORT_DIR / "repair_summary.json").write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
