project(
  'cppdict',
  'cpp',
  version : '1.0.2',
  default_options : ['warning_level=3', 'cpp_std=c++20'],
  license : 'GPL3'
)

cppdict_dep = declare_dependency(include_directories: include_directories('include'))
catch_dep = dependency('catch2-with-main', version:'>3.0.0', required : true)

install_headers(['./include/dict.hpp'], subdir : 'cppdict')
install_headers(['./include/dict/cbor.hpp'], subdir : 'cppdict/dict')

foreach test:['basic_dict_ops','stl_compatibility']
    exe = executable(test,'test/'+test+'.cpp',
                    dependencies:[cppdict_dep, catch_dep],
                    cpp_args : '-DCATCH_CONFIG_NO_POSIX_SIGNALS',
                    include_directories: include_directories('test'),
                    install: false
                    )
    test(test, exe)
endforeach

# opt-in: dict/cbor.hpp needs nlohmann_json, only its test depends on it
nlohmann_json_dep = dependency('nlohmann_json', required : false, allow_fallback : true)
if nlohmann_json_dep.found()
    cbor_exe = executable('cbor','test/cbor.cpp',
                    dependencies:[cppdict_dep, catch_dep, nlohmann_json_dep],
                    cpp_args : '-DCATCH_CONFIG_NO_POSIX_SIGNALS',
                    install: false
                    )
    test('cbor', cbor_exe)
endif

demo1 = executable('demo1','examples/demo1.cpp',
                dependencies:[cppdict_dep],
                install: false
                )

demo_visitor = executable('demo_visitor','examples/visitor.cpp',
                dependencies:[cppdict_dep],
                install: false
                )
