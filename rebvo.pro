TEMPLATE = subdirs
CONFIG += ordered
SUBDIRS = rebvolib \
          app/rebvorun\
          app/visualizer
rebvorun.depends = rebvolib
visualizer.depends = rebvolib
