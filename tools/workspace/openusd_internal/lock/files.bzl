# TODO(jwnimmer-tri) Generate this content automatically by parsing upstream's
# CMakeLists.txt files where they call `pxr_library(...)`.
FILES = {
    "pxr/base/arch": {
        "LIBRARIES": [],
        "PUBLIC_CLASSES": [
            "align",
            "attributes",
            "daemon",
            "debugger",
            "demangle",
            "env",
            "error",
            "errno",
            "fileSystem",
            "function",
            "hash",
            "library",
            "mallocHook",
            "regex",
            "stackTrace",
            "symbols",
            "systemInfo",
            "threads",
            "timing",
            "virtualMemory",
            "vsnprintf",
        ],
        "PUBLIC_HEADERS": [
            "api.h",
            "buildMode.h",
            "defines.h",
            "export.h",
            "functionLite.h",
            "hints.h",
            "inttypes.h",
            "math.h",
            "pragmas.h",
        ],
        "PRIVATE_CLASSES": [],
        "PRIVATE_HEADERS": [
            "testArchAbi.h",
            "testArchUtil.h",
        ],
        "CPPFILES": [
            "assumptions.cpp",
            "initConfig.cpp",
        ],
    },
    "pxr/base/gf": {
        "LIBRARIES": [
            "arch",
            "tf",
        ],
        "PUBLIC_CLASSES": [
            "bbox3d",
            "camera",
            "dualQuatd",
            "dualQuatf",
            "dualQuath",
            "frustum",
            "gamma",
            "half",
            "homogeneous",
            "ilmbase_half",
            "interval",
            "line",
            "line2d",
            "lineSeg",
            "lineSeg2d",
            "math",
            "matrixData",
            "matrix2d",
            "matrix2f",
            "matrix3f",
            "matrix3d",
            "matrix4f",
            "matrix4d",
            "multiInterval",
            "plane",
            "quatd",
            "quatf",
            "quath",
            "quaternion",
            "range1d",
            "range1f",
            "range2d",
            "range2f",
            "range3d",
            "range3f",
            "ray",
            "rect2i",
            "rotation",
            "size2",
            "size3",
            "transform",
            "vec2d",
            "vec2f",
            "vec2h",
            "vec2i",
            "vec3d",
            "vec3f",
            "vec3h",
            "vec3i",
            "vec4d",
            "vec4f",
            "vec4h",
            "vec4i",
        ],
        "PUBLIC_HEADERS": [
            "api.h",
            "declare.h",
            "ilmbase_halfLimits.h",
            "limits.h",
            "traits.h",
        ],
        "PRIVATE_CLASSES": [
            "ostreamHelpers",
        ],
        "PRIVATE_HEADERS": [
            "ilmbase_eLut.h",
            "ilmbase_toFloat.h",
        ],
        "CPPFILES": [],
    },
    "pxr/base/js": {
        "LIBRARIES": [
            "tf",
        ],
        "PUBLIC_CLASSES": [
            "json",
            "utils",
            "value",
        ],
        "PUBLIC_HEADERS": [
            "api.h",
            "converter.h",
            "types.h",
        ],
        "PRIVATE_CLASSES": [],
        "PRIVATE_HEADERS": [
            "rapidjson/allocators.h",
            "rapidjson/document.h",
            "rapidjson/encodedstream.h",
            "rapidjson/encodings.h",
            "rapidjson/error/en.h",
            "rapidjson/error/error.h",
            "rapidjson/filereadstream.h",
            "rapidjson/filewritestream.h",
            "rapidjson/fwd.h",
            "rapidjson/internal/biginteger.h",
            "rapidjson/internal/diyfp.h",
            "rapidjson/internal/dtoa.h",
            "rapidjson/internal/ieee754.h",
            "rapidjson/internal/itoa.h",
            "rapidjson/internal/meta.h",
            "rapidjson/internal/pow10.h",
            "rapidjson/internal/regex.h",
            "rapidjson/internal/stack.h",
            "rapidjson/internal/strfunc.h",
            "rapidjson/internal/strtod.h",
            "rapidjson/internal/swap.h",
            "rapidjson/istreamwrapper.h",
            "rapidjson/memorybuffer.h",
            "rapidjson/memorystream.h",
            "rapidjson/msinttypes/inttypes.h",
            "rapidjson/msinttypes/stdint.h",
            "rapidjson/ostreamwrapper.h",
            "rapidjson/pointer.h",
            "rapidjson/prettywriter.h",
            "rapidjson/rapidjson.h",
            "rapidjson/reader.h",
            "rapidjson/schema.h",
            "rapidjson/stream.h",
            "rapidjson/stringbuffer.h",
            "rapidjson/writer.h",
            # TODO(jwnimmer-tri) This file is missing in upstream's list.
            # Once we start auto-generating this files.bzl listing, we'll
            # need to patch upstream to declare this correctly.
            "rapidjson/internal/clzll.h",
        ],
        "CPPFILES": [],
    },
    "pxr/base/plug": {
        "LIBRARIES": [
            "arch",
            "tf",
            "js",
            "trace",
            "work",
        ],
        "PUBLIC_CLASSES": [
            "interfaceFactory",
            "notice",
            "plugin",
            "registry",
            "staticInterface",
        ],
        "PUBLIC_HEADERS": [
            "api.h",
            "thisPlugin.h",
        ],
        "PRIVATE_CLASSES": [
            "debugCodes",
            "info",
            "testPlugBase",
        ],
        "PRIVATE_HEADERS": [],
        "CPPFILES": [
            "initConfig.cpp",
        ],
    },
    "pxr/base/tf": {
        "LIBRARIES": [
            "arch",
        ],
        "PUBLIC_CLASSES": [
            "anyUniquePtr",
            "anyWeakPtr",
            "atomicOfstreamWrapper",
            "bigRWMutex",
            "bitUtils",
            "debug",
            "debugNotice",
            "denseHashMap",
            "denseHashSet",
            "diagnostic",
            "diagnosticBase",
            "diagnosticHelper",
            "diagnosticMgr",
            "dl",
            "enum",
            "envSetting",
            "error",
            "errorMark",
            "errorTransport",
            "exception",
            "expiryNotifier",
            "fastCompression",
            "fileUtils",
            "getenv",
            "hash",
            "iterator",
            "mallocTag",
            "notice",
            "nullPtr",
            "ostreamMethods",
            "pathUtils",
            "patternMatcher",
            "pointerAndBits",
            "pyLock",
            "pyObjWrapper",
            "pyTracing",
            "refBase",
            "refCount",
            "refPtr",
            "refPtrTracker",
            "regTest",
            "registryManager",
            "safeOutputFile",
            "scoped",
            "scopeDescription",
            "setenv",
            "singleton",
            "smallVector",
            "spinRWMutex",
            "stackTrace",
            "stacked",
            "status",
            "stl",
            "stopwatch",
            "stringUtils",
            "templateString",
            "tf",
            "token",
            "type",
            "typeFunctions",
            "typeNotice",
            "warning",
            "weakBase",
            "weakPtr",
            "weakPtrFacade",
        ],
        "PUBLIC_HEADERS": [
            "api.h",
            "callContext.h",
            "cxxCast.h",
            "declarePtrs.h",
            "diagnosticLite.h",
            "functionTraits.h",
            "functionRef.h",
            "hashmap.h",
            "hashset.h",
            "instantiateSingleton.h",
            "instantiateStacked.h",
            "instantiateType.h",
            "meta.h",
            "pxrCLI11/CLI11.h",
            "pxrPEGTL/pegtl.h",
            "pxrTslRobinMap/robin_growth_policy.h",
            "pxrTslRobinMap/robin_hash.h",
            "pxrTslRobinMap/robin_map.h",
            "pxrTslRobinMap/robin_set.h",
            "preprocessorUtils.h",
            "preprocessorUtilsLite.h",
            "safeTypeCompare.h",
            "span.h",
            "staticData.h",
            "staticTokens.h",
            "typeInfoMap.h",
            "type_Impl.h",
        ],
        "PRIVATE_CLASSES": [
            "atomicRenameUtil",
            "debugCodes",
            "noticeRegistry",
        ],
        "PRIVATE_HEADERS": [
            "scopeDescriptionPrivate.h",
            "pxrDoubleConversion/ieee.h",
            "pxrDoubleConversion/utils.h",
            "pxrDoubleConversion/double-conversion.h",
            "pxrDoubleConversion/bignum-dtoa.h",
            "pxrDoubleConversion/bignum.h",
            "pxrDoubleConversion/cached-powers.h",
            "pxrDoubleConversion/diy-fp.h",
            "pxrDoubleConversion/fast-dtoa.h",
            "pxrDoubleConversion/fixed-dtoa.h",
            "pxrDoubleConversion/strtod.h",
            "pxrLZ4/lz4.h",
            "pyWeakObject.h",
        ],
        "CPPFILES": [
            "initConfig.cpp",
            "preprocessorUtils.cpp",
            "pxrDoubleConversion/double-conversion.cc",
            "pxrDoubleConversion/bignum-dtoa.cc",
            "pxrDoubleConversion/bignum.cc",
            "pxrDoubleConversion/cached-powers.cc",
            "pxrDoubleConversion/diy-fp.cc",
            "pxrDoubleConversion/fast-dtoa.cc",
            "pxrDoubleConversion/fixed-dtoa.cc",
            "pxrDoubleConversion/strtod.cc",
            "pxrLZ4/lz4.cpp",
        ],
    },
    "pxr/base/trace": {
        "LIBRARIES": [
            "arch",
            "js",
            "tf",
        ],
        "PUBLIC_CLASSES": [
            "aggregateTree",
            "aggregateNode",
            "category",
            "collection",
            "collectionNotice",
            "collector",
            "counterAccumulator",
            "dataBuffer",
            "dynamicKey",
            "event",
            "eventContainer",
            "eventData",
            "eventList",
            "eventNode",
            "eventTree",
            "key",
            "reporter",
            "reporterBase",
            "reporterDataSourceBase",
            "reporterDataSourceCollection",
            "reporterDataSourceCollector",
            "serialization",
            "staticKeyData",
            "threads",
        ],
        "PUBLIC_HEADERS": [
            "api.h",
            "concurrentList.h",
            "stringHash.h",
            "trace.h",
        ],
        "PRIVATE_CLASSES": [
            "aggregateTreeBuilder",
            "eventTreeBuilder",
            "jsonSerialization",
        ],
        "PRIVATE_HEADERS": [],
        "CPPFILES": [],
    },
    "pxr/base/vt": {
        "LIBRARIES": [
            "arch",
            "tf",
            "gf",
            "trace",
        ],
        "PUBLIC_CLASSES": [
            "array",
            "dictionary",
            "functions",
            "hash",
            "streamOut",
            "types",
            "value",
        ],
        "PUBLIC_HEADERS": [
            "api.h",
            "traits.h",
            "typeHeaders.h",
            "visitValue.h",
        ],
        "PRIVATE_CLASSES": [],
        "PRIVATE_HEADERS": [],
        "CPPFILES": [],
    },
    "pxr/base/work": {
        "LIBRARIES": [
            "tf",
            "trace",
        ],
        "PUBLIC_CLASSES": [
            "detachedTask",
            "dispatcher",
            "loops",
            "reduce",
            "singularTask",
            "threadLimits",
            "utils",
        ],
        "PUBLIC_HEADERS": [
            "api.h",
            "withScopedParallelism.h",
        ],
        "PRIVATE_CLASSES": [],
        "PRIVATE_HEADERS": [],
        "CPPFILES": [],
    },
    "pxr/usd/ar": {
        "LIBRARIES": [
            "arch",
            "js",
            "plug",
            "tf",
            "vt",
        ],
        "PUBLIC_CLASSES": [
            "asset",
            "assetInfo",
            "defaultResolver",
            "defaultResolverContext",
            "definePackageResolver",
            "defineResolver",
            "filesystemAsset",
            "filesystemWritableAsset",
            "inMemoryAsset",
            "notice",
            "packageResolver",
            "packageUtils",
            "resolvedPath",
            "resolver",
            "resolverContext",
            "resolverContextBinder",
            "resolverScopedCache",
            "timestamp",
            "writableAsset",
        ],
        "PUBLIC_HEADERS": [
            "api.h",
            "ar.h",
            "defineResolverContext.h",
            "threadLocalScopedCache.h",
        ],
        "PRIVATE_CLASSES": [],
        "PRIVATE_HEADERS": [
            "debugCodes.h",
            "testenv/TestArURIResolver_plugin.h",
        ],
        "CPPFILES": [
            "debugCodes.cpp",
        ],
    },
    "pxr/usd/kind": {
        "LIBRARIES": [
            "tf",
            "plug",
        ],
        "PUBLIC_CLASSES": [
            "registry",
        ],
        "PUBLIC_HEADERS": [
            "api.h",
        ],
        "PRIVATE_CLASSES": [],
        "PRIVATE_HEADERS": [],
        "CPPFILES": [],
    },
    "pxr/usd/ndr": {
        "LIBRARIES": [
            "tf",
            "plug",
            "vt",
            "work",
            "ar",
            "sdf",
        ],
        "PUBLIC_CLASSES": [
            "debugCodes",
            "declare",
            "discoveryPlugin",
            "filesystemDiscovery",
            "filesystemDiscoveryHelpers",
            "node",
            "parserPlugin",
            "property",
            "registry",
        ],
        "PUBLIC_HEADERS": [
            "api.h",
            "nodeDiscoveryResult.h",
        ],
        "PRIVATE_CLASSES": [],
        "PRIVATE_HEADERS": [],
        "CPPFILES": [],
    },
    "pxr/usd/pcp": {
        "LIBRARIES": [
            "tf",
            "trace",
            "vt",
            "sdf",
            "work",
            "ar",
        ],
        "PUBLIC_CLASSES": [
            "arc",
            "cache",
            "changes",
            "composeSite",
            "dependency",
            "dynamicFileFormatContext",
            "dynamicFileFormatDependencyData",
            "dynamicFileFormatInterface",
            "errors",
            "expressionVariables",
            "expressionVariablesDependencyData",
            "expressionVariablesSource",
            "instanceKey",
            "iterator",
            "layerStack",
            "layerStackIdentifier",
            "mapExpression",
            "mapFunction",
            "namespaceEdits",
            "node",
            "pathTranslation",
            "primIndex",
            "propertyIndex",
            "site",
            "strengthOrdering",
            "targetIndex",
            "types",
        ],
        "PUBLIC_HEADERS": [
            "api.h",
        ],
        "PRIVATE_CLASSES": [
            "debugCodes",
            "dependencies",
            "diagnostic",
            "instancing",
            "layerStackRegistry",
            "node_Iterator",
            "primIndex_Graph",
            "primIndex_StackFrame",
            "statistics",
            "utils",
        ],
        "PRIVATE_HEADERS": [],
        "CPPFILES": [],
    },
    "pxr/usd/sdf": {
        "LIBRARIES": [
            "ar",
            "arch",
            "gf",
            "tf",
            "trace",
            "vt",
            "work",
        ],
        "PUBLIC_CLASSES": [
            "abstractData",
            "allowed",
            "assetPath",
            "attributeSpec",
            "changeBlock",
            "changeList",
            "children",
            "childrenPolicies",
            "childrenProxy",
            "childrenUtils",
            "childrenView",
            "cleanupEnabler",
            "copyUtils",
            "data",
            "declareHandles",
            "fileFormat",
            "identity",
            "layer",
            "layerOffset",
            "layerStateDelegate",
            "layerTree",
            "layerUtils",
            "listProxy",
            "listEditor",
            "listEditorProxy",
            "listOp",
            "mapEditProxy",
            "mapEditor",
            "namespaceEdit",
            "notice",
            "opaqueValue",
            "path",
            "pathExpression",
            "pathExpressionEval",
            "pathNode",
            "pathTable",
            "payload",
            "pool",
            "predicateExpression",
            "predicateLibrary",
            "primSpec",
            "propertySpec",
            "proxyPolicies",
            "proxyTypes",
            "pseudoRootSpec",
            "reference",
            "relationshipSpec",
            "schema",
            "site",
            "siteUtils",
            "spec",
            "specType",
            "textFileFormat",
            "timeCode",
            "tokens",
            "types",
            "valueTypeName",
            "variableExpression",
            "variantSetSpec",
            "variantSpec",
            "textFileFormat.tab",
        ],
        "PUBLIC_HEADERS": [
            "api.h",
            "accessorHelpers.h",
            "declareSpec.h",
            "invoke.hpp",
            "layerHints.h",
            "predicateExpressionParser.h",
            "predicateProgram.h",
            "schemaTypeRegistration.h",
        ],
        "PRIVATE_CLASSES": [
            "assetPathResolver",
            "changeManager",
            "cleanupTracker",
            "connectionListEditor",
            "debugCodes",
            "fileFormatRegistry",
            "fileIO",
            "fileIO_Common",
            "layerRegistry",
            "listOpListEditor",
            "parserHelpers",
            "parserValueContext",
            "subLayerListEditor",
            "textParserContext",
            "valueTypeRegistry",
            "variableExpressionImpl",
            "variableExpressionParser",
            "vectorListEditor",
        ],
        "PRIVATE_HEADERS": [
            "instantiatePool.h",
            "valueTypePrivate.h",
        ],
        "CPPFILES": [
            "textFileFormat.lex.cpp",
        ],
    },
    "pxr/usd/sdr": {
        "LIBRARIES": [
            "tf",
            "vt",
            "ar",
            "ndr",
            "sdf",
        ],
        "PUBLIC_CLASSES": [
            "debugCodes",
            "registry",
            "shaderMetadataHelpers",
            "shaderNode",
            "shaderProperty",
        ],
        "PUBLIC_HEADERS": [
            "api.h",
            "declare.h",
        ],
        "PRIVATE_CLASSES": [],
        "PRIVATE_HEADERS": [],
        "CPPFILES": [],
    },
    "pxr/usd/usd": {
        "LIBRARIES": [
            "arch",
            "kind",
            "pcp",
            "sdf",
            "ar",
            "plug",
            "tf",
            "trace",
            "vt",
            "work",
        ],
        "PUBLIC_CLASSES": [
            "apiSchemaBase",
            "attribute",
            "clipsAPI",
            "attributeQuery",
            "collectionAPI",
            "collectionMembershipQuery",
            "collectionPredicateLibrary",
            "common",
            "crateInfo",
            "debugCodes",
            "editContext",
            "editTarget",
            "errors",
            "flattenUtils",
            "inherits",
            "interpolation",
            "modelAPI",
            "notice",
            "object",
            "payloads",
            "prim",
            "primCompositionQuery",
            "primData",
            "primDataHandle",
            "primDefinition",
            "primFlags",
            "primRange",
            "primTypeInfo",
            "property",
            "references",
            "relationship",
            "resolveInfo",
            "resolveTarget",
            "resolver",
            "schemaBase",
            "schemaRegistry",
            "specializes",
            "stage",
            "stageCache",
            "stageCacheContext",
            "stageLoadRules",
            "stagePopulationMask",
            "timeCode",
            "tokens",
            "typed",
            "usdFileFormat",
            "usdaFileFormat",
            "usdcFileFormat",
            "usdzFileFormat",
            "variantSets",
            "zipFile",
            # TODO(jwnimmer-tri) This class is documented as "work in progress"
            # upstream and doesn't compile correctly inside Drake, so we skip
            # it. Once we start auto-generating this files.bzl listing, we'll
            # probably need to patch upstream to skip it, too.
            # "namespaceEditor",
        ],
        "PUBLIC_HEADERS": [
            "api.h",
        ],
        "PRIVATE_CLASSES": [
            "clip",
            "clipCache",
            "clipSet",
            "clipSetDefinition",
            "crateData",
            "crateFile",
            "instanceCache",
            "instanceKey",
            "integerCoding",
            "interpolators",
            "primTypeInfoCache",
            "usdzResolver",
            "valueUtils",
        ],
        "PRIVATE_HEADERS": [
            "crateDataTypes.h",
            "crateValueInliners.h",
            "listEditImpl.h",
            "wrapUtils.h",
            "testenv/TestUsdResolverChangedResolver.h",
            # TODO(jwnimmer-tri) This file is missing in upstream's list.
            # Once we start auto-generating this files.bzl listing, we'll
            # need to patch upstream to declare this correctly.
            "shared.h",
        ],
        "CPPFILES": [],
    },
    "pxr/usd/usdGeom": {
        "LIBRARIES": [
            "js",
            "tf",
            "plug",
            "vt",
            "sdf",
            "trace",
            "usd",
            "work",
        ],
        "PUBLIC_CLASSES": [
            "debugCodes",
            "tokens",
            "bboxCache",
            "constraintTarget",
            "xformCache",
            "basisCurves",
            "boundable",
            "boundableComputeExtent",
            "camera",
            "capsule",
            "capsule_1",
            "cone",
            "cube",
            "curves",
            "cylinder",
            "cylinder_1",
            "hermiteCurves",
            "imageable",
            "gprim",
            "mesh",
            "metrics",
            "modelAPI",
            "motionAPI",
            "nurbsCurves",
            "nurbsPatch",
            "plane",
            "pointBased",
            "pointInstancer",
            "points",
            "primvar",
            "primvarsAPI",
            "scope",
            "sphere",
            "subset",
            "visibilityAPI",
            "xform",
            "xformable",
            "xformOp",
            "xformCommonAPI",
        ],
        "PUBLIC_HEADERS": [
            "api.h",
        ],
        "PRIVATE_CLASSES": [
            "samplingUtils",
        ],
        "PRIVATE_HEADERS": [],
        "CPPFILES": [],
    },
    "pxr/usd/usdShade": {
        "LIBRARIES": [
            "tf",
            "vt",
            "js",
            "sdf",
            "ndr",
            "sdr",
            "usd",
            "usdGeom",
        ],
        "PUBLIC_CLASSES": [
            "connectableAPI",
            "connectableAPIBehavior",
            "coordSysAPI",
            "input",
            "material",
            "materialBindingAPI",
            "nodeDefAPI",
            "output",
            "shader",
            "shaderDefParser",
            "shaderDefUtils",
            "nodeGraph",
            "tokens",
            "udimUtils",
            "utils",
        ],
        "PUBLIC_HEADERS": [
            "api.h",
            "types.h",
        ],
        "PRIVATE_CLASSES": [],
        "PRIVATE_HEADERS": [],
        "CPPFILES": [],
    },
    "pxr/usd/usdUtils": {
        "LIBRARIES": [
            "arch",
            "tf",
            "gf",
            "sdf",
            "usd",
            "usdGeom",
            "usdShade",
        ],
        "PUBLIC_CLASSES": [
            "authoring",
            "coalescingDiagnosticDelegate",
            "conditionalAbortDiagnosticDelegate",
            "debugCodes",
            "dependencies",
            "flattenLayerStack",
            "introspection",
            "pipeline",
            "registeredVariantSet",
            "sparseValueWriter",
            "stageCache",
            "stitch",
            "stitchClips",
            "timeCodeRange",
            "usdzPackage",
        ],
        "PUBLIC_HEADERS": [
            "api.h",
        ],
        "PRIVATE_CLASSES": [
            "assetLocalization",
            "assetLocalizationDelegate",
        ],
        "PRIVATE_HEADERS": [],
        "CPPFILES": [],
    },
}
