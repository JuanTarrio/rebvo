TEMPLATE = subdirs
CONFIG += ordered
SUBDIRS = rebvolib \
          rebvorun\
          visualizer
rebvorun.subdir=app/rebvorun
visualizer.subdir=app/visualizer
rebvorun.depends = rebvolib
visualizer.depends = rebvolib
