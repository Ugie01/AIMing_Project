import tensorflow as tf

from configs.config import (
    INPUT_SIZE,
    NUM_CLASSES,
    BACKBONE_ALPHA,
    FEATURE_LAYER,
    DROPOUT_RATE
)




def build_fomo():
    # ==========================================
    # Backbone
    # ==========================================
    backbone = tf.keras.applications.MobileNetV2(
        input_shape=(
            INPUT_SIZE[0],
            INPUT_SIZE[1],
            3
        ),
        include_top=False,
        weights="imagenet",
        alpha=BACKBONE_ALPHA
    )


    # ==========================================
    # Backbone Training 설정
    # ==========================================
    #
    # FOMO는 backbone feature를
    # drone dataset에 맞게 fine tuning
    #
    backbone.trainable = True


    # ==========================================
    # FOMO Feature Map
    # ==========================================
    #
    # MobileNetV2:
    #
    # input
    # 96x96
    #
    # block_6_expand_relu
    #
    # output
    # 12x12x96
    #
    feature = backbone.get_layer(
        FEATURE_LAYER
    ).output


    # ==========================================
    # Detection Head
    # ==========================================
    x = tf.keras.layers.Dropout(
        DROPOUT_RATE
    )(feature)


    #
    # background + drone
    #
    x = tf.keras.layers.Conv2D(
        filters=NUM_CLASSES + 1,
        kernel_size=(1,1),
        padding="same",
        activation=None
    )(x)


    #
    # grid별 class probability
    #
    output = tf.keras.layers.Softmax(
        axis=-1
    )(x)


    # ==========================================
    # Model
    # ==========================================
    model = tf.keras.Model(
        inputs=backbone.input,
        outputs=output,
        name="FOMO_MobileNetV2"
    )

    return model







# import tensorflow as tf
# from configs.config import (
#     BACKBONE_ALPHA,
#     FEATURE_LAYER,
#     DROPOUT_RATE,
#     NUM_CLASSES,
#     INPUT_SIZE,
# )
#
#
#
#
# def build_fomo(
#         input_shape=(*INPUT_SIZE, 3),
#         num_classes=NUM_CLASSES,
#         alpha=BACKBONE_ALPHA,
#         dropout_rate=DROPOUT_RATE
#         # input_shape=(96, 96, 3),
#         # num_classes=1,
#         # alpha=0.35,
#         # dropout_rate=0.2
# ):
#     # ----------------------------------------
#     # Backbone
#     # ----------------------------------------
#     backbone = tf.keras.applications.MobileNetV2(
#         input_shape=input_shape,
#         include_top=False,
#         weights="imagenet",
#        alpha=alpha
#     )
#     backbone.trainable = True
#
#     # ----------------------------------------
#     # FOMO Feature Map
#     # ----------------------------------------
#     #
#     # MobileNetV2의 중간 Feature를 사용
#     #
#     # FEATURE_LAYER = "block_6_expand_relu"
#     feature = backbone.get_layer(FEATURE_LAYER).output
#
#     # ----------------------------------------
#     # Detection Head
#     # ----------------------------------------
#     x = tf.keras.layers.Dropout(
#         dropout_rate
#     )(feature)
#
#     #
#     # class + background
#     #
#     x = tf.keras.layers.Conv2D(
#         filters=num_classes + 1,
#         kernel_size=1,
#         padding="same",
#         activation=None
#     )(x)
#
#     output = tf.keras.layers.Softmax(
#         axis=-1
#     )(x)
#
#     model = tf.keras.Model(
#         inputs=backbone.input,
#         outputs=output,
#         name="FOMO_MobileNetV2"
#     )
#
#     return model