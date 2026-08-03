# ===========================
# Dataset
# ===========================
INPUT_SIZE = (96, 96)
NUM_CLASSES = 1
IMAGE_EXTENSIONS = (".jpg", ".jpeg", ".png")


# ===========================
# FOMO Model
# ===========================
BACKBONE_ALPHA = 0.35
FEATURE_LAYER = "block_6_expand_relu"
DROPOUT_RATE = 0.2


# ===========================
# Heatmap
# ===========================
GRID_SIZE = (12, 12)


# ===========================
# Training
# ===========================
BATCH_SIZE = 32
# EPOCHS = 100
EPOCHS = 30
LEARNING_RATE = 0.001