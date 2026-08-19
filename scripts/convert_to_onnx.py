#!/usr/bin/env python3
"""
Convert the rl_games PyTorch checkpoint (best_agent/) to ONNX format
without installing PyTorch. Uses raw numpy weight reconstruction + onnx builder.

Architecture (from checkpoint analysis):
  Input(4) -> Linear(4,32) -> ELU -> Linear(32,32) -> ELU -> Linear(32,2) -> Output

Includes running mean/std normalization from the state_preprocessor.

Usage:
    python3 convert_to_onnx.py
"""

import numpy as np
import onnx
from onnx import TensorProto, helper, numpy_helper
import onnxruntime as ort
import os
import sys


def load_weight(data_dir, file_idx, dtype=np.float32):
    """Load a raw tensor from the checkpoint data directory."""
    path = os.path.join(data_dir, str(file_idx))
    return np.fromfile(path, dtype=dtype)


def build_onnx_policy(data_dir, output_path):
    """
    Build the ONNX policy model from raw checkpoint weights.
    
    File mapping (from pickle analysis):
      File 0  -> policy.log_std_parameter   (2,)         [not needed for deterministic]
      File 1  -> policy.net_container.0.weight (32, 4)   [Linear layer 1]
      File 2  -> policy.net_container.0.bias   (32,)
      File 3  -> policy.net_container.2.weight (32, 32)  [Linear layer 2]
      File 4  -> policy.net_container.2.bias   (32,)
      File 5  -> policy.policy_layer.weight    (2, 32)   [Policy output head]
      File 6  -> policy.policy_layer.bias      (2,)
      File 45 -> state_preprocessor.running_mean    (4,) [float64]
      File 46 -> state_preprocessor.running_variance (4,) [float64]
    """
    
    print("=" * 60)
    print("  RL Policy -> ONNX Converter")
    print("=" * 60)
    
    # --- Load weights ---
    print("\n[1/4] Loading weights from checkpoint...")
    
    w1 = load_weight(data_dir, 1).reshape(32, 4)
    b1 = load_weight(data_dir, 2).reshape(32)
    w2 = load_weight(data_dir, 3).reshape(32, 32)
    b2 = load_weight(data_dir, 4).reshape(32)
    w3 = load_weight(data_dir, 5).reshape(2, 32)
    b3 = load_weight(data_dir, 6).reshape(2)
    
    # Running mean/std for observation normalization (stored as float64)
    running_mean = load_weight(data_dir, 45, dtype=np.float64).astype(np.float32)
    running_var = load_weight(data_dir, 46, dtype=np.float64).astype(np.float32)
    running_std = np.sqrt(np.maximum(running_var, 1e-6))
    
    print(f"  Layer 1: weight {w1.shape}, bias {b1.shape}")
    print(f"  Layer 2: weight {w2.shape}, bias {b2.shape}")
    print(f"  Policy:  weight {w3.shape}, bias {b3.shape}")
    print(f"  Running mean: {running_mean}")
    print(f"  Running std:  {running_std}")
    
    # --- Build ONNX graph ---
    print("\n[2/4] Building ONNX graph...")
    
    # Input: observation vector [batch, 4]
    obs_input = helper.make_tensor_value_info("obs", TensorProto.FLOAT, [1, 4])
    
    # Output: action vector [batch, 2]  (linear_vel, angular_vel)
    action_output = helper.make_tensor_value_info("action", TensorProto.FLOAT, [1, 2])
    
    # Create weight/bias initializers
    initializers = [
        # Normalization constants
        numpy_helper.from_array(running_mean.reshape(1, 4), name="running_mean"),
        numpy_helper.from_array(running_std.reshape(1, 4), name="running_std"),
        # Layer 1
        numpy_helper.from_array(w1.T, name="w1"),   # Transpose for MatMul: (4, 32)
        numpy_helper.from_array(b1.reshape(1, 32), name="b1"),
        # Layer 2
        numpy_helper.from_array(w2.T, name="w2"),   # (32, 32)
        numpy_helper.from_array(b2.reshape(1, 32), name="b2"),
        # Policy layer
        numpy_helper.from_array(w3.T, name="w3"),   # (32, 2)
        numpy_helper.from_array(b3.reshape(1, 2), name="b3"),
        # Clamp constants
        numpy_helper.from_array(np.array([-1.0], dtype=np.float32), name="clip_min"),
        numpy_helper.from_array(np.array([1.0], dtype=np.float32), name="clip_max"),
    ]
    
    # Build computation nodes
    nodes = [
        # Step 1: Normalize observations: (obs - mean) / std
        helper.make_node("Sub", ["obs", "running_mean"], ["norm_sub"], name="normalize_sub"),
        helper.make_node("Div", ["norm_sub", "running_std"], ["norm_obs"], name="normalize_div"),
        
        # Step 2: Linear layer 1: matmul + bias
        helper.make_node("MatMul", ["norm_obs", "w1"], ["mm1"], name="linear1_mm"),
        helper.make_node("Add", ["mm1", "b1"], ["lin1"], name="linear1_add"),
        
        # Step 3: ELU activation
        helper.make_node("Elu", ["lin1"], ["elu1"], name="elu1", alpha=1.0),
        
        # Step 4: Linear layer 2: matmul + bias
        helper.make_node("MatMul", ["elu1", "w2"], ["mm2"], name="linear2_mm"),
        helper.make_node("Add", ["mm2", "b2"], ["lin2"], name="linear2_add"),
        
        # Step 5: ELU activation
        helper.make_node("Elu", ["lin2"], ["elu2"], name="elu2", alpha=1.0),
        
        # Step 6: Policy output layer: matmul + bias
        helper.make_node("MatMul", ["elu2", "w3"], ["mm3"], name="policy_mm"),
        helper.make_node("Add", ["mm3", "b3"], ["raw_action"], name="policy_add"),
        
        # Step 7: Clamp actions to [-1, 1]
        helper.make_node("Clip", ["raw_action", "clip_min", "clip_max"], ["action"], name="clip_action"),
    ]
    
    # Create the graph
    graph = helper.make_graph(
        nodes,
        "rl_policy",
        [obs_input],
        [action_output],
        initializer=initializers,
    )
    
    # Create the model
    model = helper.make_model(
        graph,
        opset_imports=[helper.make_opsetid("", 13)],
    )
    model.ir_version = 8
    
    # --- Validate ---
    print("\n[3/4] Validating ONNX model...")
    onnx.checker.check_model(model)
    print("  ✅ ONNX model is valid!")
    
    # Save
    onnx.save(model, output_path)
    file_size = os.path.getsize(output_path)
    print(f"  📁 Saved to: {output_path} ({file_size} bytes)")
    
    # --- Test inference ---
    print("\n[4/4] Testing inference with onnxruntime...")
    
    session = ort.InferenceSession(output_path)
    
    # Test 1: Zero observation (robot at rest, no heading error)
    obs_zero = np.zeros((1, 4), dtype=np.float32)
    result = session.run(["action"], {"obs": obs_zero})[0]
    print(f"  obs=[0, 0, 0, 0] -> action={result[0]}  (should be ~forward)")
    
    # Test 2: Forward velocity, no drift
    obs_fwd = np.array([[0.3, 0.0, 0.0, 0.0]], dtype=np.float32)
    result = session.run(["action"], {"obs": obs_fwd})[0]
    print(f"  obs=[0.3, 0, 0, 0] -> action={result[0]}  (moving forward)")
    
    # Test 3: Heading error (yaw drift)
    obs_yaw = np.array([[0.0, 0.0, 0.0, 0.3]], dtype=np.float32)
    result = session.run(["action"], {"obs": obs_yaw})[0]
    print(f"  obs=[0, 0, 0, 0.3] -> action={result[0]}  (correcting yaw)")
    
    # Test 4: Lateral drift
    obs_lat = np.array([[0.0, 0.2, 0.0, 0.0]], dtype=np.float32)
    result = session.run(["action"], {"obs": obs_lat})[0]
    print(f"  obs=[0, 0.2, 0, 0] -> action={result[0]}  (correcting drift)")
    
    print("\n" + "=" * 60)
    print("  ✅ Conversion complete! Model ready for deployment.")
    print("=" * 60)
    
    return True


if __name__ == "__main__":
    data_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "best_agent", "data")
    output_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "policy.onnx")
    
    if not os.path.isdir(data_dir):
        print(f"ERROR: Checkpoint data directory not found: {data_dir}")
        sys.exit(1)
    
    success = build_onnx_policy(data_dir, output_path)
    sys.exit(0 if success else 1)
