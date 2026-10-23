# Unit tests check exact routes and clearances against the authored obstacle
# layout, so they pin the site source to it; test_site_model loads the SLAM
# map explicitly where it tests map mode. (The stack itself defaults to the
# map: config/site.yaml.)
import os

os.environ.setdefault('MASONASSIST_SITE_SOURCE', 'layout')
