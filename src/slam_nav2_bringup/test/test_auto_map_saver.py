"""Integration test for completion-triggered map persistence."""

import importlib.util
from pathlib import Path
import threading
import time
import uuid

import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from slam_toolbox.srv import SaveMap, SerializePoseGraph
from std_msgs.msg import Empty


def _load_saver_class():
    script = Path(__file__).parents[1] / 'scripts' / 'auto_map_saver.py'
    spec = importlib.util.spec_from_file_location('auto_map_saver', script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.AutoMapSaver


AutoMapSaver = _load_saver_class()


def test_rejects_unsafe_output_configuration(tmp_path):
    for unsafe_prefix in ('', '../escape', 'nested/name', '.hidden', 'has space'):
        with pytest.raises(ValueError):
            AutoMapSaver._validate_prefix(unsafe_prefix)

    with pytest.raises(ValueError):
        AutoMapSaver._prepare_directory('relative/maps')
    with pytest.raises(ValueError):
        AutoMapSaver._prepare_directory('/')

    assert AutoMapSaver._validate_prefix('robot-map_01') == 'robot-map_01'
    assert AutoMapSaver._prepare_directory(str(tmp_path)) == tmp_path.resolve()


def test_completion_saves_map_and_pose_graph_once(tmp_path):
    context = Context()
    rclpy.init(context=context)
    suffix = f't{uuid.uuid4().hex}'
    completion_topic = f'/test/{suffix}/exploration_complete'
    save_service = f'/test/{suffix}/save_map'
    serialize_service = f'/test/{suffix}/serialize_map'
    requests = {'map': [], 'pose_graph': []}

    server = Node(f'mock_slam_toolbox_{suffix}', context=context)

    def save_map(request, response):
        basename = Path(request.name.data)
        requests['map'].append(basename)
        Path(f'{basename}.yaml').write_text('image: mock.pgm\n', encoding='utf-8')
        Path(f'{basename}.pgm').write_bytes(b'P5\n1 1\n255\n\x00')
        response.result = SaveMap.Response.RESULT_SUCCESS
        return response

    def serialize_map(request, response):
        basename = Path(request.filename)
        requests['pose_graph'].append(basename)
        Path(f'{basename}.posegraph').write_bytes(b'mock graph')
        Path(f'{basename}.data').write_bytes(b'mock data')
        response.result = SerializePoseGraph.Response.RESULT_SUCCESS
        return response

    server.create_service(SaveMap, save_service, save_map)
    server.create_service(SerializePoseGraph, serialize_service, serialize_map)
    completion_qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    completion_publisher = server.create_publisher(
        Empty, completion_topic, completion_qos)

    parameters = [
        Parameter('map_directory', value=str(tmp_path)),
        Parameter('map_prefix', value='mock_map'),
        Parameter('completion_topic', value=completion_topic),
        Parameter('save_map_service', value=save_service),
        Parameter('serialize_pose_graph_service', value=serialize_service),
        Parameter('serialize_pose_graph', value=True),
        Parameter('service_wait_timeout', value=5.0),
    ]
    saver = AutoMapSaver(context=context, parameter_overrides=parameters)
    executor = MultiThreadedExecutor(num_threads=2, context=context)
    executor.add_node(server)
    executor.add_node(saver)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        discovery_deadline = time.monotonic() + 5.0
        while completion_publisher.get_subscription_count() == 0:
            assert time.monotonic() < discovery_deadline, (
                'completion subscriber not discovered')
            time.sleep(0.02)

        completion_publisher.publish(Empty())
        completion_publisher.publish(Empty())

        save_deadline = time.monotonic() + 5.0
        while not saver.done:
            assert time.monotonic() < save_deadline, 'map saver did not finish'
            time.sleep(0.02)

        # Publish again after completion so this checks the node's one-shot
        # state, rather than relying on the depth-one QoS queue to drop a burst.
        completion_publisher.publish(Empty())
        time.sleep(0.1)

        assert saver.save_succeeded
        assert len(requests['map']) == 1
        assert len(requests['pose_graph']) == 1
        assert requests['map'][0] == requests['pose_graph'][0]
        assert requests['map'][0].parent == tmp_path.resolve()
        assert requests['map'][0].name.startswith('mock_map_')
        assert Path(f"{requests['map'][0]}.yaml").is_file()
        assert Path(f"{requests['map'][0]}.pgm").is_file()
        assert Path(f"{requests['map'][0]}.posegraph").is_file()
        assert Path(f"{requests['map'][0]}.data").is_file()
    finally:
        executor.shutdown(timeout_sec=2.0)
        spin_thread.join(timeout=2.0)
        executor.remove_node(saver)
        executor.remove_node(server)
        saver.destroy_node()
        server.destroy_node()
        rclpy.shutdown(context=context)
