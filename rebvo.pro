TEMPLATE = subdirs
CONFIG += ordered
SUBDIRS = rebvolib \
          rebvorun\
          visualizer\
          kf_visualizer
rebvorun.subdir=app/rebvorun
visualizer.subdir=app/visualizer
kf_visualizer.subdir=app/kf_visualizer
rebvorun.depends = rebvolib
visualizer.depends = rebvolib
kf_visualizer.depends = rebvolib
