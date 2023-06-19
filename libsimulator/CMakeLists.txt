################################################################################
# Build libsimulator
################################################################################
add_library(simulator STATIC
    src/AABB.cpp
    src/AABB.hpp
    src/AgentExitSystem.hpp
    src/Clonable.hpp
    src/CollisionGeometry.cpp
    src/CollisionGeometry.hpp
    src/DTriangulation.cpp
    src/DTriangulation.hpp
    src/Ellipse.cpp
    src/Ellipse.hpp
    src/Enum.hpp
    src/Events.hpp
    src/GCFMModel.cpp
    src/GCFMModel.hpp
    src/GCFMModelBuilder.cpp
    src/GCFMModelBuilder.hpp
    src/GenericAgent.hpp
    src/GeometricFunctions.hpp
    src/Geometry.cpp
    src/Geometry.hpp
    src/GeometryBuilder.cpp
    src/GeometryBuilder.hpp
    src/Graph.hpp
    src/Journey.cpp
    src/Journey.hpp
    src/LineSegment.cpp
    src/LineSegment.hpp
    src/Logger.cpp
    src/Logger.hpp
    src/Macros.hpp
    src/Mathematics.cpp
    src/Mathematics.hpp
    src/NeighborhoodSearch.hpp
    src/OperationalDecisionSystem.hpp
    src/OperationalModel.hpp
    src/Point.cpp
    src/Point.hpp
    src/Polygon.cpp
    src/Polygon.hpp
    src/RoutingEngine.cpp
    src/RoutingEngine.hpp
    src/Simulation.hpp
    src/SimulationClock.cpp
    src/SimulationClock.hpp
    src/SimulationError.hpp
    src/Stage.cpp
    src/Stage.hpp
    src/StrategicalDesicionSystem.hpp
    src/TacticalDecisionSystem.hpp
    src/TemplateHelper.hpp
    src/Tracing.cpp
    src/Tracing.hpp
    src/Triangle.cpp
    src/Triangle.hpp
    src/UniqueID.hpp
    src/Util.hpp
    src/VelocityModel.cpp
    src/VelocityModel.hpp
    src/VelocityModelBuilder.cpp
    src/VelocityModelBuilder.hpp
)
target_compile_options(simulator PRIVATE
    ${COMMON_COMPILE_OPTIONS}
)
target_compile_definitions(simulator PUBLIC
    JPSCORE_VERSION="${PROJECT_VERSION}"
)
target_link_libraries(simulator PUBLIC
    Boost::boost
    poly2tri
    fmt::fmt
    CGAL::CGAL
)
target_link_options(simulator PUBLIC 
    $<$<AND:$<OR:$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:AppleClang>>,$<BOOL:${BUILD_WITH_ASAN}>>:-fsanitize=address>
)
target_include_directories(simulator PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
set_property(TARGET simulator PROPERTY INTERPROCEDURAL_OPTIMIZATION ${USE_IPO})
set_property(TARGET simulator PROPERTY INTERPROCEDURAL_OPTIMIZATION_DEBUG OFF)

################################################################################
# libsimulator unit tests
################################################################################
if (BUILD_TESTS)
    add_executable(libsimulator-tests
        test/TestAABB.cpp
        test/TestBasicPrimitiveTests.cpp
        test/TestGraph.cpp
        test/TestLineSegment.cpp
        test/TestNeighborhoodSearch.cpp
        test/TestSimulationClock.cpp
        test/TestUniqueID.cpp
    )

    target_link_libraries(libsimulator-tests PRIVATE
        GTest::gtest
        GTest::gtest_main
        simulator
    )

    target_compile_options(libsimulator-tests PRIVATE
        ${COMMON_COMPILE_OPTIONS}
    )

    set_property(TARGET libsimulator-tests PROPERTY INTERPROCEDURAL_OPTIMIZATION ${USE_IPO})
    set_property(TARGET libsimulator-tests PROPERTY INTERPROCEDURAL_OPTIMIZATION_DEBUG OFF)
endif()