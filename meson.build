project('sim6502', 'c', default_options : ['c_std=c89', 'werror=true'])

srcs = [
    'src/sim6502.c',
    'src/main.c',
]

cargs = ['-pedantic-errors', '-Wall', '-Wextra', '-fno-strict-aliasing']

config_data = configuration_data()
config_data.set('version_str', '"0.0.1"')
config_data.set('version_major', 0)
config_data.set('version_minor', 0)
config_data.set('version_patch', 0)
configure_file(
    input : 'src/config_sim6502.h.in',
    output : 'config_sim6502.h',
    configuration : config_data
)
config_inc = include_directories('.')

cc = meson.get_compiler('c')
m_dep = cc.find_library('m', required : false)

executable('sim6502',
    sources : srcs,
    c_args : cargs,
    include_directories : [config_inc],
    dependencies : m_dep
)
