#include "stdafx.h"
#include "FaceModel.h"
#include "FeaturePointExtractor.h"

const std::string filenameAverageMesh = "averageMesh.off";
const std::string filenameAverageMeshFeaturePoints = "averageMesh_features.points";
const std::string filenameBasisShape = "ShapeBasis.matrix";
const std::string filenameBasisAlbedo = "AlbedoBasis.matrix";
const std::string filenameBasisExpression = "ExpressionBasis.matrix";
const std::string filenameStdDevShape = "StandardDeviationShape.vec";
const std::string filenameStdDevExpression = "StandardDeviationExpression.vec";

Eigen::MatrixXf discardEvery4thRow(const Eigen::Ref<const Eigen::MatrixXf>& matrix) {
	assert(matrix.rows() % 4 == 0 && "matrix rows need to be a multiple of 4");
	Eigen::Index numBlocks = matrix.rows() / 4;
	Eigen::MatrixXf result(3 * numBlocks, matrix.cols());
	for (int i = 0; i < numBlocks; i++) {
		result.middleRows(3 * i, 3) = matrix.middleRows(4 * i, 3);
	}
	return result;
}

FaceModel::FaceModel(const std::string& baseDir) {
	// load average shape
	m_averageMesh = loadOFF(baseDir + filenameAverageMesh);
	m_averageMesh.vertices /= 1000000.0f;
	// load average shape feature points
	FeaturePointExtractor averageFeatureExtractor(baseDir + filenameAverageMeshFeaturePoints, nullptr);
	m_averageFeaturePoints = averageFeatureExtractor.m_points;

	unsigned int nVertices = getNumVertices();

	// load shape basis
	std::vector<float> shapeBasisRaw = loadBinaryVector(baseDir + filenameBasisShape);
	unsigned int nEigenVec = shapeBasisRaw.size() / (4 * nVertices);
	Eigen::Map<Eigen::MatrixXf> shapeBasis4(shapeBasisRaw.data(), 4 * nVertices, nEigenVec);
	m_shapeBasis = discardEvery4thRow(shapeBasis4);

	// load albedo basis
	std::vector<float> albedoBasisRaw = loadBinaryVector(baseDir + filenameBasisAlbedo);
	if (albedoBasisRaw.size() != shapeBasisRaw.size()) {
		std::cout << "ERROR: Expected albedo basis to be the same size as shape basis." << std::endl;
		exit(1);
	}
	Eigen::Map<Eigen::MatrixXf> albedoBasis4(albedoBasisRaw.data(), 4 * nVertices, nEigenVec);
	m_albedoBasis = discardEvery4thRow(albedoBasis4);

	// load expression basis
	std::vector<float> expressionBasisRaw = loadBinaryVector(baseDir + filenameBasisExpression);
	unsigned int nExpr = expressionBasisRaw.size() / (4 * nVertices);
	Eigen::Map<Eigen::MatrixXf> expressionBasis4(expressionBasisRaw.data(), 4 * nVertices, nExpr);
	m_expressionBasis = discardEvery4thRow(expressionBasis4);

	// not needed yet
	/*
	auto shapeDevRaw = new float[nEigenVec];
	LoadVector(filenameStdDevShape, shapeDevRaw, nEigenVec);

	auto expressionDevRaw = new float[nExpr];
	LoadVector(filenameStdDevExpression, expressionDevRaw, nExpr);
	*/
}

Eigen::VectorXf FaceModel::computeShape(const FaceParameters& params) const
{
	assert(params.alpha.rows() == m_shapeBasis.cols() && "face parameter alpha has incorrect size");
	return m_averageMesh.vertices + m_shapeBasis * params.alpha;
}

Eigen::Matrix4Xi FaceModel::computeColors(const FaceParameters& params) const
{
	assert(params.beta.rows() == m_albedoBasis.cols() && "face parameter beta has incorrect size");
	// interpolate RGB values as floats
	Eigen::Matrix3Xf colorsRGB = m_averageMesh.vertexColors.topRows<3>().cast<float>();
	// reshape from matrix (3, numVertices) to vector (3 * numVertices)
	Eigen::Map<Eigen::VectorXf> flatColorsRGB(colorsRGB.data(), 3 * getNumVertices());

	flatColorsRGB += m_albedoBasis * params.beta;

	// convert back to RGBA int representation
	Eigen::Matrix4Xi result(4, getNumVertices());
	result.topRows<3>() = colorsRGB.cast<int>();
	result.row(3).setConstant(255);
	return result;
}

FaceParameters FaceModel::computeShapeAttribute(const FaceParameters& params, float age, float weight, float gender) const
{
	FaceParameters outparams = params;
	// load attribute age alpha +50 * older
	Eigen::VectorXf age_attribute(200);
	age_attribute << 0.017355, -0.034198, 0.0077516, -0.01159, 0.0005499, -0.011274, 0.010518, 0.025153, -0.013924, -0.017966, -0.025759, -0.017126, 0.01749, -0.0049113, 0.023187, -0.018283, 0.0070943, 0.0018614, -0.0015066, 0.012837, 0.012817, 0.014319, -0.019735, 0.014488, -0.027718, -0.018185, -0.0099453, 0.010539, 0.0084646, 0.0072581, -0.000406, -0.0081624, -0.0027972, 0.013811, 0.011792, -0.021877, -0.015496, -0.014063, 0.0072784, 0.0083727, -0.01303, 0.00080588, 0.0085552, -0.0054485, 0.0031326, -0.012376, -0.018324, 0.0066565, 2.3123e-05, -0.0089248, -0.015026, 0.014599, -0.0052527, -0.0031472, -0.0023898, 0.0093744, -0.013422, 0.0026074, 0.0070199, 0.0060414, 0.0020569, -0.011855, 0.0014679, 0.0021108, 0.010274, -0.0027192, -0.0065645, -0.0069326, 0.0044031, -0.0029327, 0.00081024, -0.004243, -0.00074079, 0.0030895, 0.0073547, 0.00422, -0.0013479, -0.0051695, 0.00053903, -0.0047337, 0.0010555, -0.0070548, -0.0023792, -0.0070698, -0.0068006, -0.011157, -0.0059115, -0.003554, 0.0018075, -0.0085133, 0.0031075, -0.0033138, -0.0074831, -0.0075626, -0.0064035, 0.00016418, 0.0085761, -0.00092182, 0.0078562, -0.0073328, -0.004364, 0.0027872, -0.0063015, -0.0058222, -0.00035885, -0.00090252, -0.0016063, -0.0056352, -0.0019139, -0.0016239, -0.0032307, -0.001624, 0.0013865, -0.0010914, 0.0011, 0.00057639, 0.0019971, 0.0021058, -0.0060394, -0.0020238, 0.0040678, -0.011674, 0.0049451, 0.0027056, 0.010256, 0.0026385, 0.0012109, 0.0045915, 0.005882, -0.0063201, 0.00048387, 0.0063002, 0.001455, -0.0039404, 0.0027386, -0.001703, 0.0012045, -0.0052172, 0.0045235, 0.0011824, -0.0019752, -0.0030432, 0.0018381, -0.0017199, -0.0035327, -0.00032697, -0.001309, 0.0065152, -0.0025409, -0.0043674, -0.0035899, -0.0034098, -0.00016286, 0.0038444, 0.0047698, 0.0047522, 0.00076151, -0.00094393, -0.0010848, -0.003358, -0.010807, 0.0024576, 0.0010408, -0.00014859, -0.006437, -0.0024533, 0.0052521, -0.003024, -0.0057812, -0.0035245, 0.0035467, -0.001752, -0.00044278, 0.00038587, -0.00062015, 0.0010627, 0.00035227, 0.0067316, 0.001487, -0.0012553, -0.00023609, -0.001204, -0.007436, 0.0030112, 0.0013344, -0.00035531, -0.00038852, -0.0042449, -0.0013478, 0.0030729, 0.00065055, 0.0029813, -0.0029558, 0.0026678, 0.0056648, 0.00015243, 0.00027627, -0.002153, -0.00022908, -6.9749e-08;

	// load attribute weight alpha +30* fatter
	Eigen::VectorXf weight_attribute(200);
	weight_attribute << 0.044351, 0.0013907, -0.0020525, 0.020258, -0.0029185, -0.022733, 0.0067942, -0.0007726, 0.0054469, 0.019335, 0.017035, 0.015141, -0.013596, 0.0048483, 0.00074636, -0.0021607, -0.0086522, -0.0025784, 0.0045336, 0.0012447, -0.018608, 0.0053882, 0.0078904, 0.0058695, 0.0044992, 0.012867, 0.0057868, 0.0060062, 0.013482, 0.0047342, 0.011996, 0.010441, 0.0077854, 0.0022859, 0.0042956, 0.0093804, 0.0093139, 0.0025465, -0.00082258, 0.0022546, 0.0098654, 0.0082443, -0.0066921, -0.0069501, -0.0013659, -0.0027028, 0.0041211, 0.0001581, 0.0019449, -0.00406, 0.010697, -0.0068584, -0.010674, -0.0054776, -0.0063793, 0.0069608, 0.0009438, 0.0014058, -0.0052412, 0.0026599, 0.00013753, 0.0086275, 0.0072897, -0.0012705, -0.011211, 0.004887, -0.0099241, -0.0056025, -0.00027208, 0.0046485, -0.0039607, 0.00058753, 0.0019238, -8.0841e-05, 0.0046176, 0.0021697, 0.01151, 0.0031984, 0.0093146, -0.0095353, -0.0029325, -0.00058046, 0.0010014, -0.0039476, 0.0056857, 0.011672, -0.0023487, -0.0022226, -0.0045376, 0.0055685, -0.00076162, 0.0023651, 0.0041864, -0.0034049, -0.0065708, 0.0085388, -0.0039355, -0.0054047, 0.0011788, 0.0077542, 0.0045653, -0.014171, -0.00029504, 0.0048552, -0.0032344, -0.0063228, -0.0059384, 0.0025629, -0.0011107, 0.006464, 0.0032442, -0.0023974, 0.0010987, -0.00053787, -0.0053354, -0.0038626, 0.0018981, -0.00036749, -0.0028358, -0.0037226, 0.0036574, 0.0025723, 0.00069458, -0.0061373, 0.0046408, 0.0046881, -0.00075388, 0.00045124, -0.0032309, 0.00038934, -0.0032816, -0.0039997, -0.013988, 0.0019064, 0.00219, 0.0051255, -0.0030605, 0.00098398, -0.0033401, -0.0037673, 0.006907, -0.0060079, -0.0033867, 0.0019007, -0.0074113, -0.0067972, -0.0073397, -0.013346, 0.012121, 0.0031945, 0.00093088, 1.3438e-05, -0.00043136, -0.00932, -0.0047609, 0.0029339, -0.0037496, 0.008826, 0.0016341, -0.001484, 0.011428, -0.0032988, 0.012512, 0.0039352, 0.00079125, 0.0018652, -0.009041, 0.0077252, 0.004884, -0.0028475, 0.0026759, -0.0043115, -0.0037104, -0.0048171, 0.0064952, 0.0068225, -0.0051827, 0.0032262, 0.0028933, -0.003991, -0.0008606, 0.0071075, 0.0059774, 0.0015468, 0.0072185, -0.0040749, 0.0036053, 0.0053654, 0.0022245, 0.00055251, 0.0023773, -0.0057749, 0.0029474, -0.0021497, -0.0032109, 0.0022427, -0.0084225, -0.0049743, 0.0027353, 6.1845e-08;

	// load attribute gender alpha +5* manner
	Eigen::VectorXf gender_attribute(200);
	gender_attribute << 0.38832, 0.14996, -0.34608, -0.10604, -0.12026, 0.36505, 0.0068739, -0.18151, 0.059577, -0.18345, -0.088721, -0.18571, 0.16947, -0.12282, 0.021034, -0.013913, 0.020309, 0.025513, 0.11601, -0.14296, 0.061797, 0.085725, 0.026433, -0.0052271, -0.0013855, -0.14896, -0.04612, -0.085454, -0.22277, -0.075828, -0.10519, -0.057262, 0.029241, -0.090424, -0.047449, -0.1966, -0.089687, 0.03619, 0.04954, 0.15556, -0.007982, -0.12521, 0.041978, 0.050437, -0.010352, 0.033437, 0.012643, 0.11748, -0.040278, 0.032599, -0.30281, -0.044535, 0.18167, -0.034015, 0.13515, -0.14396, 0.011355, 0.023236, -0.0072029, 0.0041677, 0.083037, -0.054984, -0.067704, 0.14355, 0.070192, 0.051023, -0.034218, -0.018309, 0.073423, -0.089829, 0.13017, 0.063107, -0.011346, -0.0019437, -0.094402, 0.053771, -0.056251, -0.057488, 0.026084, 0.042868, -0.091269, -0.0059091, -0.028782, -0.058945, -0.094643, -0.085037, 0.040094, -0.045598, -0.0049116, -0.027994, 0.024147, -0.10711, -0.0017302, 0.066473, 0.047006, 0.045922, -0.052184, -0.035478, -0.10993, -0.050262, 0.074209, 0.13432, 0.074271, -0.031878, -0.012765, 0.11381, -0.10744, -0.001553, -0.015233, -0.035086, 0.032528, 0.00071718, -0.048987, 0.024672, -0.062269, 0.13786, -0.080217, 0.034213, 0.0096885, 0.0073342, -0.020128, 0.042779, -0.013687, 0.094579, -0.046009, 0.0058711, 0.031606, 0.050234, -0.0057513, 0.080516, 0.064308, 0.091744, -0.030459, -0.00055161, 0.045656, -0.037836, -0.014615, -0.10267, 0.0084999, 0.067655, -0.039774, -0.022349, -0.02869, 0.056761, 0.03387, -0.047138, 0.054013, 0.051372, -0.12439, -0.05448, 0.055591, 0.018482, 0.060171, 0.02447, 0.020255, -0.013624, 0.034204, -0.0089235, -0.021847, 0.010521, -0.11039, 0.0070307, 0.030231, 0.027633, 0.069004, -0.030215, 0.052522, -0.063828, -0.088316, 0.044961, -0.037345, -0.054678, -0.040143, 0.08135, -0.035715, -0.039459, 0.093668, 0.0232, -0.059914, 0.0019242, 0.010752, -0.03391, 0.024546, -0.04136, -0.0057477, -0.01921, -0.035253, 0.020519, -0.097296, -0.057941, 0.0046735, -0.0097141, -0.0081759, -0.084275, 0.045288, 0.022289, 0.024549, 0.01065, -0.073065, -1.4301e-06;

	// load attribute height alpha 
	Eigen::VectorXf height_attribute(200);
	height_attribute << 0.012269, -0.014972, 0.023427, -0.030798, 0.015781, 0.014637, -0.0053564, 0.00056268, -0.0058421, -0.014402, -0.00037659, 0.0046289, -0.0013334, 0.0081884, 0.005643, 0.0049922, 0.0043122, 0.0032663, -0.0098733, 0.0037353, 0.010524, -0.014441, -0.013814, -0.0036654, 0.0090254, -0.00786, 0.0065716, -0.0040186, 0.0064585, 0.00073369, -0.0093873, 0.0051652, -0.014847, 0.0080093, -0.0023533, 0.014164, -0.0052239, -0.0080147, -0.0042344, -0.018345, -0.012683, -0.0017831, 0.0097908, 0.005555, 0.0043595, -0.001146, -0.0059975, -0.0096099, 0.00032476, 0.0017427, 0.028304, 0.019416, 0.0017503, 0.01664, -0.00099394, 0.0067908, -0.0041914, -0.0072744, 0.010084, -0.00075899, -0.01703, -0.0036808, -0.0014255, -0.018292, -0.0030549, -0.014197, 0.02572, 0.0064962, -0.0093564, 0.0074793, -0.011539, -0.007575, 0.0024957, -0.0038174, 0.0080731, -0.010822, -0.0065963, 0.0061666, -0.014642, 0.0048664, 0.010666, 0.0026489, 0.0061831, 0.0091422, 0.0093207, -0.0034054, -0.0025345, 0.0081168, 0.0048398, -0.0079739, -0.0045627, 0.0094917, -0.0088698, -0.0017957, 0.0053196, -0.015861, 0.013128, 0.018358, 0.0073761, 0.00076072, -0.017191, -0.0019497, -0.012421, -0.0041578, 0.0070755, -0.0075711, 0.017173, -0.00038584, 0.0030383, -0.0014846, -0.0085008, 0.0039588, 0.0072789, 0.0020759, 0.01655, -0.0044912, 0.0022552, -0.0075263, -0.0033966, 0.0049001, -0.0062781, -0.010371, -0.004166, -0.0066866, 0.0008759, -0.0026883, -0.0035121, -0.011201, -0.00041162, -0.0098681, -0.0038078, -0.012997, 0.017376, 0.00014344, -0.011475, 0.001698, 0.010917, 0.0085375, -0.0045813, -0.0014596, -0.005331, 0.0068464, 0.0057593, -0.0071435, 0.0054429, 0.014647, 0.0041013, 0.0020729, 0.0047978, 0.0014123, -0.0076053, -0.0014605, -0.0054828, 0.0045129, -0.00068413, -0.0042156, -0.0063913, -0.011513, 0.00013507, 0.00054097, 0.0024249, -0.0025915, -0.017798, -0.011093, -0.0080166, 0.0017648, 0.0044813, -0.00028168, 0.0082877, 0.001033, -0.0010629, 0.0081358, 0.0032398, -0.0024912, 0.0017818, -0.0022931, -0.0066424, -0.0068656, 0.0013389, 0.0060137, -0.00066961, -0.0085524, -0.0075241, 0.004021, -0.0098846, 0.0070657, 0.0013695, -0.0067392, 0.008806, 0.0053669, -0.0045337, 0.0032961, -0.0068182, 0.013719, -0.002098, -0.0040329, 0.0085071, 0.0011306, 0.0022516, 1.3307e-07;

	outparams.alpha = params.alpha + age * age_attribute + weight * weight_attribute + gender * gender_attribute;
	return outparams;
}

// Utility function to primitively load a file in the STOFF format.
const Mesh FaceModel::loadOFF(const std::string& filename) const {
	std::ifstream in(filename, std::ifstream::in);
	if (!in) {
		std::cout << "ERROR:\tCan not open file: " << filename << std::endl;
		exit(1);
	}

	std::string line;
	std::getline(in, line); // Header, should be STOFF
	std::getline(in, line); // contains number of vertices and triangles
	std::istringstream headerStream(line);
	int nVertices, nTriangles;
	headerStream >> nVertices >> nTriangles;

	Mesh mesh;
	mesh.vertices.resize(3 * nVertices);
	mesh.vertexColors.resize(4, nVertices);
	mesh.triangles.resize(3, nTriangles);

	std::cout << "  vertices ..." << std::endl;
	float x, y, z;
	int r, g, b, a;
	for (int i = 0; i < nVertices; i++) {
		std::getline(in, line);
		std::istringstream lineStream(line);
		lineStream >> x >> y >> z >> r >> g >> b >> a;
		mesh.vertices(3 * i + 0) = x;
		mesh.vertices(3 * i + 1) = y;
		mesh.vertices(3 * i + 2) = z;
		mesh.vertexColors.col(i) << r, g, b, a;
	}

	std::cout << "  triangles ..." << std::endl;
	int count, v1, v2, v3;
	for (int i = 0; i < nTriangles; i++) {
		std::getline(in, line);
		std::istringstream lineStream(line);
		lineStream >> count >> v1 >> v2 >> v3;

		if (count != 3) {
			std::cerr << "WARNING: Can only process triangles, found face with " << count << " vertices while reading " << filename << std::endl;
			v1 = v2 = v3 = 0;
		}

		mesh.triangles.col(i) << v1, v2, v3;
	}

	in.close();

	return mesh;
}

std::vector<float> FaceModel::loadBinaryVector(const std::string &filename) const {
	std::ifstream in(filename, std::ifstream::in | std::ifstream::binary);
	if (!in) {
		std::cout << "ERROR:\tCan not open file: " << filename << std::endl;
		exit(1);
	}
	unsigned int numberOfEntries;
	in.read((char *)&numberOfEntries, sizeof(unsigned int));

	std::vector<float> result(numberOfEntries);
	in.read((char*)result.data(), numberOfEntries * sizeof(float));

	in.close();
	return result;
}
