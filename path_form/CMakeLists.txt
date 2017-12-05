
SET( PLUGINS
  footbot_test
  footbot_static
)

# create a library module for each plugin and link libstage to each

foreach( PLUGIN ${PLUGINS} )
  ADD_LIBRARY( ${PLUGIN} MODULE ${PLUGIN}.cc behaviour.cpp motor_schema.cpp)
endforeach( PLUGIN )

set_source_files_properties( ${PLUGINS} PROPERTIES 
  COMPILE_FLAGS "${FLTK_CFLAGS}" 
)

foreach( PLUGIN ${PLUGINS} )
  TARGET_LINK_LIBRARIES( ${PLUGIN} stage ${OPENGL_LIBRARIES} )
endforeach( PLUGIN )

# delete the "lib" prefix from the plugin libraries
SET_TARGET_PROPERTIES( ${PLUGINS} PROPERTIES PREFIX "" )

# install in <prefix>/lib
install( TARGETS ${PLUGINS} DESTINATION ${PROJECT_PLUGIN_DIR}) 

