#!/usr/bin/env python3
"""
Generate an rqt_graph-style node/topic graph for the Social Navigation ROS2 project.
Produces a PDF and PNG in the current directory.

Usage:
    python3 generate_rqt_graph.py
    # Outputs: rqt_graph.pdf, rqt_graph.png
"""

import subprocess
import sys
import os

# ── Ensure graphviz python package is available ──
try:
    import graphviz
except ImportError:
    subprocess.check_call([sys.executable, "-m", "pip", "install", "graphviz"])
    import graphviz


def build_graph():
    dot = graphviz.Digraph(
        "rqt_graph",
        format="png",
        engine="dot",
        graph_attr={
            "rankdir": "LR",
            "dpi": "150",
            "fontsize": "11",
            "fontname": "Helvetica",
            "bgcolor": "#1e1e1e",
            "pad": "0.5",
            "nodesep": "0.6",
            "ranksep": "1.8",
            "label": "ROS2 Computational Graph — Social Navigation Project\\n(rqt_graph style)",
            "labelloc": "t",
            "fontcolor": "white",
        },
    )

    # ── Style helpers ──
    node_style = {
        "shape": "ellipse",
        "style": "filled",
        "fillcolor": "#3b82f6",
        "fontcolor": "white",
        "fontname": "Helvetica-Bold",
        "fontsize": "11",
        "penwidth": "1.5",
        "color": "#60a5fa",
    }
    topic_style = {
        "shape": "box",
        "style": "filled,rounded",
        "fillcolor": "#1f2937",
        "fontcolor": "#e5e7eb",
        "fontname": "Helvetica",
        "fontsize": "9",
        "penwidth": "1.0",
        "color": "#4b5563",
    }
    gz_style = {
        "shape": "box3d",
        "style": "filled",
        "fillcolor": "#7c3aed",
        "fontcolor": "white",
        "fontname": "Helvetica-Bold",
        "fontsize": "12",
        "penwidth": "2.0",
        "color": "#a78bfa",
    }
    rviz_style = {
        "shape": "box3d",
        "style": "filled",
        "fillcolor": "#059669",
        "fontcolor": "white",
        "fontname": "Helvetica-Bold",
        "fontsize": "12",
        "penwidth": "2.0",
        "color": "#34d399",
    }
    edge_pub = {"color": "#60a5fa", "penwidth": "1.2", "arrowsize": "0.8"}
    edge_sub = {"color": "#f97316", "penwidth": "1.2", "arrowsize": "0.8"}
    edge_srv = {"color": "#f43f5e", "style": "dashed", "penwidth": "1.4", "arrowsize": "0.8"}

    # ═══════════════════════════════════════════════
    # NODES (ROS2 nodes)
    # ═══════════════════════════════════════════════
    nodes = [
        "gazebo",
        "ros_gz_bridge",
        "map_publisher",
        "pointcloud_mapper",
        "camera_view_360",
        "localization_node",
        "move_humans",
        "human_detector_red",
        "global_planner",
        "social_nav_planner",
        "live_visualization_node",
        "human_case_controller",
        "rviz2",
    ]

    for n in nodes:
        if n == "gazebo":
            dot.node(n, "Gazebo\nHarmonic", **gz_style)
        elif n == "rviz2":
            dot.node(n, "RViz2", **rviz_style)
        elif n == "ros_gz_bridge":
            dot.node(n, "ros_gz_bridge", **{**node_style, "fillcolor": "#6d28d9", "color": "#a78bfa"})
        else:
            dot.node(n, n.replace("_", "_\n") if len(n) > 16 else n, **node_style)

    # ═══════════════════════════════════════════════
    # TOPICS
    # ═══════════════════════════════════════════════
    topics = [
        "/clock",
        "/cmd_vel",
        "/odom",
        "/scan",
        "/camera/front/image",
        "/camera/right/image",
        "/camera/back/image",
        "/camera/left/image",
        "/map",
        "/robot_pose",
        "/global_path",
        "/goal_pose",
        "/detected_humans",
        "/human_velocities",
        "/human_ground_truth",
        "/weight_zones",
        "/replan_request",
        "/weighted_grid_viz",
        "/path_markers",
        "/global_planner_status",
        "/social_nav_markers",
        "/goal_reached",
        "/camera_view_360/panorama",
        "/scan_pointcloud",
        "/map_pointcloud",
        "/detected_humans_markers",
        "/viz/gt_occupancy",
        "/viz/perception_occupancy",
        "/viz/robot_estimate",
        "/viz/gt_markers",
        "/viz/social_circles",
        "/tf",
    ]

    for t in topics:
        dot.node(t, t, **topic_style)

    # ═══════════════════════════════════════════════
    # EDGES — Publishers (node → topic)
    # ═══════════════════════════════════════════════
    publishers = [
        # Gazebo → bridge
        ("gazebo", "ros_gz_bridge", {"label": "sim", **edge_pub}),
        # Bridge publishes to ROS topics
        ("ros_gz_bridge", "/clock", edge_pub),
        ("ros_gz_bridge", "/odom", edge_pub),
        ("ros_gz_bridge", "/scan", edge_pub),
        ("ros_gz_bridge", "/camera/front/image", edge_pub),
        ("ros_gz_bridge", "/camera/right/image", edge_pub),
        ("ros_gz_bridge", "/camera/back/image", edge_pub),
        ("ros_gz_bridge", "/camera/left/image", edge_pub),
        # map_publisher
        ("map_publisher", "/map", edge_pub),
        # pointcloud_mapper
        ("pointcloud_mapper", "/scan_pointcloud", edge_pub),
        ("pointcloud_mapper", "/map_pointcloud", edge_pub),
        ("pointcloud_mapper", "/tf", edge_pub),
        # camera_view_360
        ("camera_view_360", "/camera_view_360/panorama", edge_pub),
        # localization_node
        ("localization_node", "/robot_pose", edge_pub),
        # move_humans
        ("move_humans", "/detected_humans", edge_pub),
        ("move_humans", "/human_velocities", edge_pub),
        ("move_humans", "/human_ground_truth", edge_pub),
        # human_case_controller (alternative to move_humans)
        ("human_case_controller", "/detected_humans", {**edge_pub, "style": "dashed"}),
        ("human_case_controller", "/human_velocities", {**edge_pub, "style": "dashed"}),
        # human_detector_red
        ("human_detector_red", "/detected_humans", {**edge_pub, "style": "dotted"}),
        ("human_detector_red", "/human_velocities", {**edge_pub, "style": "dotted"}),
        ("human_detector_red", "/detected_humans_markers", edge_pub),
        # global_planner
        ("global_planner", "/global_path", edge_pub),
        ("global_planner", "/weighted_grid_viz", edge_pub),
        ("global_planner", "/path_markers", edge_pub),
        ("global_planner", "/global_planner_status", edge_pub),
        # social_nav_planner
        ("social_nav_planner", "/cmd_vel", edge_pub),
        ("social_nav_planner", "/weight_zones", edge_pub),
        ("social_nav_planner", "/replan_request", edge_pub),
        ("social_nav_planner", "/social_nav_markers", edge_pub),
        ("social_nav_planner", "/goal_reached", edge_pub),
        # live_visualization_node
        ("live_visualization_node", "/viz/gt_occupancy", edge_pub),
        ("live_visualization_node", "/viz/perception_occupancy", edge_pub),
        ("live_visualization_node", "/viz/robot_estimate", edge_pub),
        ("live_visualization_node", "/viz/gt_markers", edge_pub),
        ("live_visualization_node", "/viz/social_circles", edge_pub),
    ]

    # ═══════════════════════════════════════════════
    # EDGES — Subscribers (topic → node)
    # ═══════════════════════════════════════════════
    subscribers = [
        # cmd_vel → bridge → Gazebo
        ("/cmd_vel", "ros_gz_bridge", edge_sub),
        ("ros_gz_bridge", "gazebo", {"label": "cmd", **edge_sub}),
        # pointcloud_mapper
        ("/scan", "pointcloud_mapper", edge_sub),
        ("/odom", "pointcloud_mapper", edge_sub),
        # camera_view_360
        ("/camera/front/image", "camera_view_360", edge_sub),
        ("/camera/right/image", "camera_view_360", edge_sub),
        ("/camera/back/image", "camera_view_360", edge_sub),
        ("/camera/left/image", "camera_view_360", edge_sub),
        # localization_node
        ("/odom", "localization_node", edge_sub),
        ("/scan", "localization_node", edge_sub),
        ("/map", "localization_node", edge_sub),
        # human_detector_red
        ("/camera/front/image", "human_detector_red", edge_sub),
        ("/camera/right/image", "human_detector_red", edge_sub),
        ("/camera/back/image", "human_detector_red", edge_sub),
        ("/camera/left/image", "human_detector_red", edge_sub),
        ("/scan", "human_detector_red", edge_sub),
        ("/robot_pose", "human_detector_red", edge_sub),
        # global_planner
        ("/map", "global_planner", edge_sub),
        ("/robot_pose", "global_planner", edge_sub),
        ("/goal_pose", "global_planner", edge_sub),
        ("/weight_zones", "global_planner", edge_sub),
        ("/replan_request", "global_planner", edge_sub),
        ("/detected_humans", "global_planner", edge_sub),
        ("/human_velocities", "global_planner", edge_sub),
        # social_nav_planner
        ("/global_path", "social_nav_planner", edge_sub),
        ("/robot_pose", "social_nav_planner", edge_sub),
        ("/detected_humans", "social_nav_planner", edge_sub),
        ("/human_velocities", "social_nav_planner", edge_sub),
        ("/scan", "social_nav_planner", edge_sub),
        # live_visualization_node
        ("/map", "live_visualization_node", edge_sub),
        ("/robot_pose", "live_visualization_node", edge_sub),
        ("/detected_humans", "live_visualization_node", edge_sub),
        ("/human_velocities", "live_visualization_node", edge_sub),
        ("/human_ground_truth", "live_visualization_node", edge_sub),
        # RViz subscriptions
        ("/weighted_grid_viz", "rviz2", edge_sub),
        ("/path_markers", "rviz2", edge_sub),
        ("/social_nav_markers", "rviz2", edge_sub),
        ("/camera_view_360/panorama", "rviz2", edge_sub),
        ("/scan_pointcloud", "rviz2", edge_sub),
        ("/map_pointcloud", "rviz2", edge_sub),
        ("/detected_humans_markers", "rviz2", edge_sub),
        ("/viz/gt_occupancy", "rviz2", edge_sub),
        ("/viz/perception_occupancy", "rviz2", edge_sub),
        ("/viz/robot_estimate", "rviz2", edge_sub),
        ("/viz/gt_markers", "rviz2", edge_sub),
        ("/viz/social_circles", "rviz2", edge_sub),
        ("/map", "rviz2", edge_sub),
        ("/tf", "rviz2", edge_sub),
    ]

    # Service calls (dashed red)
    services = [
        ("move_humans", "ros_gz_bridge", {"label": "/world/set_pose\n(service)", **edge_srv}),
        ("human_case_controller", "ros_gz_bridge", {"label": "/world/set_pose\n(service)", **edge_srv}),
    ]

    for src, dst, attrs in publishers:
        dot.edge(src, dst, **attrs)
    for src, dst, attrs in subscribers:
        dot.edge(src, dst, **attrs)
    for src, dst, attrs in services:
        dot.edge(src, dst, **attrs)

    # ═══════════════════════════════════════════════
    # LEGEND
    # ═══════════════════════════════════════════════
    with dot.subgraph(name="cluster_legend") as legend:
        legend.attr(
            label="Legend",
            style="filled,rounded",
            fillcolor="#111827",
            fontcolor="white",
            fontname="Helvetica-Bold",
            color="#374151",
        )
        legend.node("leg_node", "ROS2 Node", **node_style)
        legend.node("leg_topic", "/topic", **topic_style)
        legend.node("leg_gz", "External\n(Gazebo)", **gz_style)
        legend.node("leg_rviz", "Visualization\n(RViz)", **rviz_style)
        legend.node("leg_a", " ", shape="point", width="0")
        legend.node("leg_b", " ", shape="point", width="0")
        legend.node("leg_c", " ", shape="point", width="0")
        legend.node("leg_d", " ", shape="point", width="0")
        legend.edge("leg_a", "leg_b", label="  publishes", **edge_pub)
        legend.edge("leg_c", "leg_d", label="  subscribes", **edge_sub)

    return dot


def main():
    out_dir = os.path.dirname(os.path.abspath(__file__))
    dot = build_graph()

    # Render PNG
    out_path = os.path.join(out_dir, "rqt_graph")
    dot.render(out_path, cleanup=True)
    print(f"✅  rqt_graph.png  →  {out_path}.png")

    # Also render PDF
    dot.format = "pdf"
    dot.render(out_path, cleanup=True)
    print(f"✅  rqt_graph.pdf  →  {out_path}.pdf")


if __name__ == "__main__":
    main()
