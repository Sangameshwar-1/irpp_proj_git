#!/bin/bash
# =============================================================================
#  Build the IRPP Docker image
# =============================================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "========================================="
echo "  Building IRPP Social Navigation Image  "
echo "========================================="

docker compose build

echo ""
echo "✅ Build complete!"
echo "   Image: irpp-social-nav:latest"
echo ""
echo "   Run with:  ./run.sh"
