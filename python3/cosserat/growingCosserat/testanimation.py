# coding: utf-8

# splib is available here: https://github.com/SofaDefrost/STLIB
import splib3


## This function is called by the animation manager at each timestep, the factor
## is updated to indicate how much has progress the animation.
## The factor is between [0, 1] 0 is the begin of the animation, 1 is the end.
def myanimate(target, factor, src, dst):
    t = [(dst[0] - src[0]) * factor + src[0],
         (dst[1] - src[1]) * factor + src[1],
         (dst[2] - src[2]) * factor + src[2]]

    print("I'm updating the position using: ", factor, " new pos is", t)
    target.position = t


def createScene(rootNode):
    from splib3.animation import animate, AnimationManager

    ## This will create a single AnimationManager python script controller that will animate all the
    ## animation on the scene.
    AnimationManager(rootNode)

    # The object
    t = rootNode.createObject("MechanicalObject", position=[0, 0, 0])
    t.showObject = True
    t.showObjectScale = 10

    # This fire up an animation,
    # The movement is computed using the "myanimate" function
    # The target of the animation is the 't' object
    # This animation will have 0.5 second duration (simulation time)
    # 'ping-pong' means that the animation will repeat to-from.
    animate(myanimate, {"target": t, "src": [-1.0, -1.0, 0], "dst": [+1.0, 2.0, 0]}, duration=5.0, mode="pingpong")