#!/usr/bin/env bash
###############################################################################
# BEVFusion-ROS TensorRT environment script  (dedup + idempotent version)
###############################################################################

# ─────────────────────────────────────────────────────────────────────────────
# 0.  Make the script idempotent
# ─────────────────────────────────────────────────────────────────────────────
if [[ -n "${_BEV_ENV_ALREADY_SOURCED:-}" ]]; then
    return 0          # nothing to do – we’ve run before
fi
export _BEV_ENV_ALREADY_SOURCED=1

# helper: prepend DIR to VAR only if it isn’t there yet
prepend_unique () {
    local var="$1" dir="$2"
    if [[ -z "$dir" ]]; then return; fi
    case ":${!var}:" in
        *":$dir:"*) ;;                     # already present – skip
        *) eval export "$var=\"$dir:${!var}\"" ;;
    esac
}

# ─────────────────────────────────────────────────────────────────────────────
# 1.  Paths to toolkits (same as your original script)
# ─────────────────────────────────────────────────────────────────────────────
# TensorRT 8.5 in Docker
export TensorRT_Lib=/opt/TensorRT/lib
export TensorRT_Inc=/opt/TensorRT/include
export TensorRT_Bin=/opt/TensorRT/bin

# CUDA 11.8
export CUDA_HOME=/usr/local/cuda-11.8
export CUDA_Lib=$CUDA_HOME/lib64
export CUDA_Inc=$CUDA_HOME/include
export CUDA_Bin=$CUDA_HOME/bin

# cuDNN is in CUDA lib path
export CUDNN_Lib=$CUDA_HOME/lib64

# Model / precision selectors
export DEBUG_MODEL=resnet50          # resnet50 / resnet50int8 / swint
export DEBUG_PRECISION=fp16         # fp16 / int8
export DEBUG_DATA=example-data
export USE_Python=OFF

# ─────────────────────────────────────────────────────────────────────────────
# 2.  Sanity-check toolkit executables
# ─────────────────────────────────────────────────────────────────────────────
if [[ ! -x "${TensorRT_Bin}/trtexec" ]]; then
    echo "❌ TensorRT executable not found at: ${TensorRT_Bin}/trtexec"
    return 1
fi
if [[ ! -x "${CUDA_Bin}/nvcc" ]]; then
    echo "❌ CUDA compiler not found at: ${CUDA_Bin}/nvcc"
    return 1
fi

# ─────────────────────────────────────────────────────────────────────────────
# 3.  Echo configuration summary
# ─────────────────────────────────────────────────────────────────────────────
cat <<EOF
==========================================================
||  MODEL       : $DEBUG_MODEL
||  PRECISION   : $DEBUG_PRECISION
||  DATA        : $DEBUG_DATA
||  USE Python  : $USE_Python
||--------------------------------------------------------
||  TensorRT    : $TensorRT_Lib
||  CUDA        : $CUDA_HOME
||  cuDNN       : $CUDNN_Lib
==========================================================
EOF

# ─────────────────────────────────────────────────────────────────────────────
# 4.  Build directory & (optional) Python headers
# ─────────────────────────────────────────────────────────────────────────────
export BuildDirectory="$(pwd)/build"

if [[ "$USE_Python" == "ON" ]]; then
    export Python_Inc=$(python3 -c "import sysconfig,sys;print(sysconfig.get_path('include'))")
    export Python_Lib=$(python3 -c "import sysconfig,sys;print(sysconfig.get_config_var('LIBDIR'))")
    export Python_Soname=$(python3 - <<'PY'
import sysconfig, re, sys
print(re.sub(r'\.a$', '.so', sysconfig.get_config_var('LIBRARY')))
PY
)
    echo "Python_Inc      : $Python_Inc"
    echo "Python_Lib      : $Python_Lib"
    echo "Python_Soname   : $Python_Soname"
fi

# ─────────────────────────────────────────────────────────────────────────────
# 5.  Export search paths (deduplicated)
# ─────────────────────────────────────────────────────────────────────────────
prepend_unique PATH "$TensorRT_Bin"
prepend_unique PATH "$CUDA_Bin"

prepend_unique LD_LIBRARY_PATH "$TensorRT_Lib"
prepend_unique LD_LIBRARY_PATH "$CUDA_Lib"
prepend_unique LD_LIBRARY_PATH "$CUDNN_Lib"
prepend_unique LD_LIBRARY_PATH "$BuildDirectory"

prepend_unique PYTHONPATH "$BuildDirectory"

# ─────────────────────────────────────────────────────────────────────────────
# 6.  Detect GPU SM (unchanged)
# ─────────────────────────────────────────────────────────────────────────────
if [[ -f "tool/cudasm.sh" ]]; then
    echo "Detecting current GPU SM…"
    . "tool/cudasm.sh"
    echo "Current CUDA SM : $cudasm"
    export CUDASM=$cudasm
fi

echo "✅ Environment configuration complete!"
export ConfigurationStatus=Success
