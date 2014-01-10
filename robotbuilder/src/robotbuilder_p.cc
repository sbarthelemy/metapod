#include "robotbuilder_p.hh"
#include "txttemplate.hh"
#include <iostream>
#include <cassert>
#include <algorithm>
#include <cctype>
#include <sstream>
#include <boost/tokenizer.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/format.hpp>
#include <set>

#if BOOST_VERSION <=104000
# include <boost/filesystem/convenience.hpp>
#endif

#include <boost/algorithm/string/join.hpp>
#include <metapod/tools/constants.hh>
#ifdef _MSC_VER
# include <stdio.h> // for _set_output_format
#endif

namespace {

template <typename T>
std::string to_string(T x) {
  std::ostringstream ss;
  ss << x;
  return ss.str();
}

bool IsLetter(char c) {
  return ('a' <= c && c <= 'z') || ('A' <= c && c <= 'Z');
}

bool IsLetterOrNumberOrUnderscore(char c) {
  return IsLetter(c) || ('0' <= c && c <= '9') || ( c == '_');
}

bool IsNotLetterOrNumberOrUnderscore(char c) {
  return !IsLetterOrNumberOrUnderscore(c);
}

std::set<std::string> GetReservedKeywords() {
  std::set<std::string> s;
  s.insert("asm");
  s.insert("auto");
  s.insert("bool");
  s.insert("break");
  s.insert("case");
  s.insert("catch");
  s.insert("char");
  s.insert("class");
  s.insert("const");
  s.insert("const_cast");
  s.insert("continue");
  s.insert("default");
  s.insert("delete");
  s.insert("do");
  s.insert("double");
  s.insert("dynamic_cast");
  s.insert("else");
  s.insert("enum");
  s.insert("explicit");
  s.insert("export");
  s.insert("extern");
  s.insert("false");
  s.insert("float");
  s.insert("for");
  s.insert("friend");
  s.insert("goto");
  s.insert("if");
  s.insert("inline");
  s.insert("int");
  s.insert("long");
  s.insert("mutable");
  s.insert("namespace");
  s.insert("new");
  s.insert("operator");
  s.insert("private");
  s.insert("protected");
  s.insert("public");
  s.insert("register");
  s.insert("reinterpret_cast");
  s.insert("return");
  s.insert("short");
  s.insert("signed");
  s.insert("sizeof");
  s.insert("static");
  s.insert("static_cast");
  s.insert("struct");
  s.insert("switch");
  s.insert("template");
  s.insert("this");
  s.insert("throw");
  s.insert("true");
  s.insert("try");
  s.insert("typedef");
  s.insert("typeid");
  s.insert("typename");
  s.insert("union");
  s.insert("unsigned");
  s.insert("using");
  s.insert("virtual");
  s.insert("void");
  s.insert("volatile");
  s.insert("wchar_t");
  s.insert("while");
  s.insert("and");
  s.insert("and_eq");
  s.insert("bitand");
  s.insert("bitor");
  s.insert("compl");
  s.insert("not");
  s.insert("not_eq");
  s.insert("or");
  s.insert("or_eq");
  s.insert("xor");
  s.insert("xor_eq");
  return s;

}

static
bool IsReservedKeyword(const std::string &name) {
  static const std::set<std::string> reserved_keywords(GetReservedKeywords());
  return (std::find(reserved_keywords.begin(), reserved_keywords.end(), name)
          != reserved_keywords.end());
}

static
bool IsValidIdentifier(const std::string &name) {
  return (!name.empty() &&
          IsLetter(name[0]) &&
          (std::find_if(++(name.begin()), name.end(),
                           IsNotLetterOrNumberOrUnderscore)
              == name.end()) &&
          !IsReservedKeyword(name));
}

// coin up a link/node name from joint name and body name.
// currently simply return the body name.
static
std::string node_name(const std::string &/*joint_name*/,
                      const std::string &body_name) {
  return body_name;
}

static
std::string node_name(const metapod::RobotModel &model, int link_id) {
  return node_name(model.joint_name(link_id),
                   model.body_name(link_id));
}

// text of the template source files
extern "C" const char config_hh[];
extern "C" const size_t config_hh_len;
extern "C" const char init_hh[];
extern "C" const size_t init_hh_len;
extern "C" const char init_cc[];
extern "C" const size_t init_cc_len;

}

namespace metapod {

RobotBuilderP::RobotBuilderP()
    : is_initialized_(false),
      root_body_name_("NP")
{}

RobotBuilderP::~RobotBuilderP()
{}

RobotBuilder::Status RobotBuilderP::set_directory(const std::string &directory)
{
  directory_ = directory;
  return RobotBuilder::STATUS_SUCCESS;
}

RobotBuilder::Status RobotBuilderP::set_name(const std::string &name) {
  if (!::IsValidIdentifier(name)) {
    std::cerr
        << "ERROR: name \"" << name << "\" is invalid."
        << std::endl;
    return RobotBuilder::STATUS_FAILURE;
  }
  name_ = name;
  return RobotBuilder::STATUS_SUCCESS;
}

RobotBuilder::Status RobotBuilderP::set_libname(const std::string &name) {
  if (!::IsValidIdentifier(name)) {
    std::cerr
        << "ERROR: libname \"" << name << "\" is invalid."
        << std::endl;
    return RobotBuilder::STATUS_FAILURE;
  }
  libname_ = name;
  return RobotBuilder::STATUS_SUCCESS;
}

RobotBuilder::Status
RobotBuilderP::set_root_body_name(const std::string &name) {
  if (is_initialized_) {
    std::cerr
        << "ERROR: one cannot call set_use_dof_index() after having called"
        << " addLink()" << std::endl;
    return RobotBuilder::STATUS_FAILURE;
  }
  root_body_name_ = name;
  return RobotBuilder::STATUS_SUCCESS;
}

RobotBuilder::Status RobotBuilderP::set_license(const std::string &text) {
  license_ = text;
  return RobotBuilder::STATUS_SUCCESS;
}

void RobotBuilderP::WriteTemplate(const std::string &output_filename,
                                  const std::string &input_template,
                                  const ReplMap &replacements) const {
  assert(is_initialized_);
  std::stringstream output_path;
  output_path << directory_ << "/" << output_filename;
  std::ofstream output_stream;
  output_stream.open(output_path.str().c_str());
  output_stream << metapod::TxtTemplate(input_template).Format(replacements);
  output_stream.close();
}

RobotBuilder::Status RobotBuilderP::Init() {
  assert(!is_initialized_);
  is_initialized_ = true;
  return RobotBuilder::STATUS_SUCCESS;
}

RobotBuilder::Status RobotBuilderP::RequireJointVariable(
    const std::vector<std::string> &joints_names,
    unsigned int nb_dof,
    int dof_index) {
  if (model_.RequireVariable(joints_names, nb_dof, dof_index))
    return RobotBuilder::STATUS_SUCCESS;
  return RobotBuilder::STATUS_FAILURE;
}

RobotBuilder::Status RobotBuilderP::AddLink(
    const std::string &parent_body_name,
    const std::string &joint_name,
    const RobotBuilder::Joint &joint,
    const Eigen::Matrix3d &R_joint_parent,
    const Eigen::Vector3d &r_parent_joint,
    const std::string &body_name,
    double body_mass,
    const Eigen::Vector3d &body_center_of_mass,
    const Eigen::Matrix3d &body_rotational_inertia) {
  if (!is_initialized_) {
    RobotBuilder::Status status = Init();
    if (status == RobotBuilder::STATUS_FAILURE)
      return RobotBuilder::STATUS_FAILURE;
  }
  // find an homonym joint
  int joint_homonym_id = model_.FindLinkByJointName(joint_name);
  if (joint_homonym_id != NO_NODE) {
    std::cerr
        << "ERROR: there is already a joint named '" << joint_name << "'"
        << std::endl;
    return RobotBuilder::STATUS_FAILURE;
  }
  // check body_name
  if (body_name == root_body_name_) {
    std::cerr
        << "ERROR: the root body (mass-less galilean frame) is also named '"
        << body_name << "'" << std::endl;
    return RobotBuilder::STATUS_FAILURE;
  }
  // find an homonym body
  int body_homonym_id = model_.FindLinkByBodyName(body_name);
  if (body_homonym_id != NO_NODE) {
    std::cerr
        << "ERROR: there is already a body named '" << body_name << "'"
        << std::endl;
    return RobotBuilder::STATUS_FAILURE;
  }
  // check node name is ok.
  std::string node_name = ::node_name(joint_name, body_name);
  if (!::IsValidIdentifier(node_name)) {
    std::cerr
        << "ERROR: node name \"" << node_name << "\" is invalid." << std::endl;
    return RobotBuilder::STATUS_FAILURE;
  }
  // TODO: check joint_Xt_E is a real rotation matrix

  // find the parent
  int parent_id = NO_NODE;
  if (parent_body_name == root_body_name_)
    parent_id = NO_PARENT;
  else
    parent_id = model_.FindLinkByBodyName(parent_body_name);
  if (parent_id == NO_NODE) {
    std::cerr
        << "ERROR: could not find parent body named '"
        << parent_body_name << "'. "
        << "Check the name and the order you add bodies in." << std::endl;
    return RobotBuilder::STATUS_FAILURE;
  }

  if (model_.nb_children(parent_id) >= MAX_NB_CHILDREN_PER_NODE) {
    std::cerr
        << "ERROR: a node cannot have more than " << MAX_NB_CHILDREN_PER_NODE
        << " children per node." << std::endl;
    return RobotBuilder::STATUS_FAILURE;
  }
  if (model_.nb_links() >= MAX_NB_JOINTS) {
    std::cerr
        << "ERROR: a model cannot have more than " << MAX_NB_JOINTS
        << " joints." << std::endl;
    return RobotBuilder::STATUS_FAILURE;
    }
  // ensure there is a joint variable for this joint
  bool ok = model_.RequireVariable(std::vector<std::string>(1, joint_name),
                                   joint.nb_dof());
  if (!ok) {
    return RobotBuilder::STATUS_FAILURE;
  }
  // add the link for real
  int link_id = model_.nb_links();
  model_.AddLink(Link(link_id,
                      parent_id,
                      joint_name,
                      joint,
                      R_joint_parent,
                      r_parent_joint,
                      body_name,
                      body_mass,
                      body_center_of_mass,
                      body_rotational_inertia));
  return RobotBuilder::STATUS_SUCCESS;
}

void RobotBuilderP::WriteLink(int link_id, const ReplMap &replacements,
                              TmpStreams &out) const {
  Eigen::IOFormat comma_fmt(Eigen::StreamPrecision, Eigen::DontAlignCols,
                            ", ", ", ");
  const int parent_id = model_.parent_id(link_id);
  ReplMap repl(replacements);
  repl["node_id"] = ::to_string(link_id);
  repl["node_name"] = ::node_name(model_, link_id);
  repl["dof_index"] = ::to_string(model_.dof_index(link_id));
  repl["joint_type"] = model_.joint(link_id).joint_class();
  repl["joint_rotation_type"] = model_.joint(link_id).rotation_class();
  repl["joint_name"] = model_.joint_name(link_id);

  const Eigen::Matrix3d &R_joint_parent = model_.R_joint_parent(link_id);
  if (R_joint_parent.isApprox(Eigen::Matrix3d::Identity())) {
    repl["R_joint_parent_type"] = "Spatial::RotationMatrixIdentity";
    repl["X_joint_parent_type"] = "Spatial::TransformT<Spatial::RotationMatrixIdentity>";
    repl["R_joint_parent"] = "Spatial::RotationMatrixIdentity()";
  } else {
    repl["R_joint_parent_type"] = "Spatial::RotationMatrix";
    repl["X_joint_parent_type"] = "Spatial::Transform";
    std::stringstream ss0;
    ss0 << "matrix3dMaker("
        << model_.R_joint_parent(link_id).format(comma_fmt)
        << ")";
    repl["R_joint_parent"] = ss0.str();
  }
  std::stringstream ss1;
  ss1 << "Vector3d("
      << model_.r_parent_joint(link_id).format(comma_fmt)
      << ")";
  repl["r_parent_joint"] = ss1.str();
  repl["body_name"] = model_.body_name(link_id);
  repl["body_mass"] = ::to_string(model_.body_mass(link_id));
  std::stringstream ss2;
  ss2 << "Vector3d("
      << model_.body_center_of_mass(link_id).format(comma_fmt)
      << ")";
  repl["body_center_of_mass"] = ss2.str();
  std::stringstream ss3;
  ss3 << "matrix3dMaker("
      << model_.body_rotational_inertia(link_id).format(comma_fmt)
      << ")";
  repl["body_rotational_inertia"] = ss3.str();
  repl["parent_id"] = ::to_string(parent_id);

  // fill childX_id and children_idt
  std::stringstream children_idt;
  for (int i = 0; i<MAX_NB_CHILDREN_PER_NODE; ++i) {
    std::stringstream key;
    key << "child" << i << "_id";
    repl[key.str()] = ::to_string(model_.child_id(link_id, i));
    children_idt << ", " << i;
  }
  repl["children_idt"] = children_idt.str();
  repl["sibling_id"] = ::to_string(model_.right_sibling_id(link_id));
  bool is_last_link = (link_id == model_.nb_links()-1);

  const TxtTemplate tpl0(
      "    @node_name@ = @node_id@");
  out.nodeid_enum_definition << tpl0.Format(repl);
  if (!is_last_link)
    out.nodeid_enum_definition << ",\n";

  const TxtTemplate tpl0bis(
      "    case @node_id@:\n"
      "      return boost::fusion::at_c<@node_id@>(nodes);");
  out.map_node_id_to_rtnode << tpl0bis.Format(repl);
  if (!is_last_link)
    out.map_node_id_to_rtnode << "\n";

  const TxtTemplate tpl1(
      "\n"
      "  // Joint: @joint_name@\n"
      "  // Body: @body_name@\n"
      "  class @LIBRARY_NAME@_DLLAPI Node@node_id@ : public RtNodeImpl<Node@node_id@> {\n"
      "  public:\n"
      "    Node@node_id@();\n"
      "    static const int id = @node_id@;\n"
      "    static const std::string joint_name;\n"
      "    static const std::string body_name;\n"
      "    static const @X_joint_parent_type@ Xt;\n"
      "    static const int q_idx = @dof_index@;\n"
      "    typedef @joint_type@ Joint;\n"
      "    static const int parent_id = @parent_id@;\n"
      "    static const int child0_id = @child0_id@;\n"
      "    static const int child1_id = @child1_id@;\n"
      "    static const int child2_id = @child2_id@;\n"
      "    static const int child3_id = @child3_id@;\n"
      "    static const int child4_id = @child4_id@;\n"
      "    static const int sibling_id = @sibling_id@;\n"
      "    typedef boost::mpl::int_<@parent_id@> parent_idt;\n"
      "    typedef boost::mpl::int_<@child0_id@> child0_idt;\n"
      "    typedef boost::mpl::int_<@sibling_id@> sibling_idt;\n"
      "    typedef boost::mpl::list_c<int@children_idt@> children_idt;\n"

      "    Spatial::TransformT<Spatial::rm_mul_op<@joint_rotation_type@, @R_joint_parent_type@>::rm> sXp;\n"
      "    Eigen::Matrix<FloatType, 6, Joint::NBDOF> joint_F; // used by crba\n"
      "    Joint joint;\n"
      "    Body body;\n"
      "  };\n");
  out.node_type_definitions << tpl1.Format(repl);

  const TxtTemplate tpl2(
                         "      Node@node_id@");
  out.nodes_type_list << tpl2.Format(repl);
  if (!is_last_link)
    out.nodes_type_list << ",\n";

  // fill bits for init.cc
  const TxtTemplate tpl4(
      "const std::string @ROBOT_CLASS_NAME@::Node@node_id@::joint_name = std::string(\"@joint_name@\");\n"
      "const std::string @ROBOT_CLASS_NAME@::Node@node_id@::body_name = std::string(\"@body_name@\");\n"
      "const @X_joint_parent_type@ @ROBOT_CLASS_NAME@::Node@node_id@::Xt = @X_joint_parent_type@(\n"
      "    @R_joint_parent@,\n"
      "    @r_parent_joint@);\n"
      "@ROBOT_CLASS_NAME@::Node@node_id@::Node@node_id@():\n"
      "  joint(@joint_args@) {}\n\n");
  repl["joint_args"] = model_.joint(link_id).ctor_args();
  out.init_nodes << tpl4.Format(repl);
  const TxtTemplate tpl5(
      "    spatialInertiaMaker(\n"
      "        @body_mass@,\n"
      "        @body_center_of_mass@,\n"
      "        @body_rotational_inertia@),\n");
  out.init_inertias << tpl5.Format(repl);
}

// an iteration of the depth-first traversal
void RobotBuilderP::GenCrbaLink(std::ostream &os,
                                std::set<IntPair> &written_blocks,
                                int i) const {
  assert(i != NO_CHILD);
  // dft_discover(i)
  os << boost::format("Iic[%1%] = robot.inertias[%1%];\n") % i;
  // recursively explore children, if any
  if (model_.left_child_id(i) != NO_CHILD)
    GenCrbaLink(os, written_blocks, model_.left_child_id(i));
  // dft_finish(i)
  if (model_.parent_id(i) != NO_PARENT) {
    os << boost::format("Iic[%1%] = Iic[%1%] + get_node<%2%>(robot).sXp.applyInv(Iic[%2%]);\n") % model_.parent_id(i) % i;
  }
  // We create one F per node because F depends on the joint number of dof.
  // We could instead reuse the same F, either using a buffer and an Eigen::Map,
  // or one F for all the joints sharing the same number of dof.
  os << boost::format("Eigen::Matrix<FloatType, 6, Robot::Node%1%::Joint::NBDOF> F%1% = Iic[%1%] * get_node<%1%>(robot).joint.S;\n") % i;

  int idx_i = model_.dof_index(i);
  const bool first_write = written_blocks.insert(std::make_pair(idx_i, idx_i)).second;
  const std::string op(first_write ? "=" : "+=");
  os << boost::format("H.template block<Robot::Node%1%::Joint::NBDOF, Robot::Node%1%::Joint::NBDOF>(\n"
                      "    Robot::Node%1%::q_idx, Robot::Node%1%::q_idx).noalias() %2% get_node<%1%>(robot).joint.S.transpose() * F%1%;\n") % i % op;
  int j = i;
  int parent_j = model_.parent_id(j);
  while (parent_j != NO_PARENT) {
    os << boost::format("F%1% = get_node<%2%>(robot).sXp.mulMatrixTransposeBy(F%1%);\n") % i % j;
    j = parent_j;
    int idx_j = model_.dof_index(j);

    const bool do_assign = (idx_i < idx_j) ?
        written_blocks.insert(std::make_pair(idx_i, idx_j)).second :
        written_blocks.insert(std::make_pair(idx_j, idx_i)).second;
    const std::string op(do_assign ? "=" : "+=");
    const std::string pre_mul((idx_i == idx_j) ? "2 *" : "");
    if (idx_i < idx_j) {
      os << boost::format("H.template block< Robot::Node%1%::Joint::NBDOF, Robot::Node%2%::Joint::NBDOF >(\n"
                          "    Robot::Node%1%::q_idx, Robot::Node%2%::q_idx ).noalias()\n"
                          "         %3% %4% F%1%.transpose() * get_node<%2%>(robot).joint.S.S();\n") % i % j % op % pre_mul;
    } else {
      os << boost::format("H.template block< Robot::Node%2%::Joint::NBDOF, Robot::Node%1%::Joint::NBDOF >(\n"
                          "    Robot::Node%2%::q_idx, Robot::Node%1%::q_idx ).noalias()\n"
                          "         %3% %4% get_node<%2%>(robot).joint.S.S().transpose() * F%1%;\n") % i % j % op % pre_mul;
    }
    parent_j = model_.parent_id(j);
  }
  // recursively explore sibling, if any
  if (model_.right_sibling_id(i) != NO_CHILD)
    GenCrbaLink(os, written_blocks, model_.right_sibling_id(i));
}


std::string RobotBuilderP::GenCrba(const ReplMap &replacements) const {
  std::ostringstream os;
  const TxtTemplate tpl(
      "template<typename Robot, typename Derived>\n"
      "struct crba_cg;\n"
      "template<typename Derived>\n"
      "struct crba_cg<@ROBOT_NAME@, Derived> {\n"
      "  typedef @ROBOT_NAME@ Robot;\n"
      "  Robot &robot;\n" // TODO: add const
      "  Eigen::MatrixBase<Derived> &H;\n"
      "  Spatial::Inertia Iic[Robot::NBBODIES];\n"
      "  crba_cg(Robot &robot, Eigen::MatrixBase<Derived> &H) :\n"
      "    robot(robot), H(H) {\n"
      "    assert(H.rows() ==  Robot::NBDOF);\n"
      "    assert(H.cols() ==  Robot::NBDOF);\n"
      "  }\n"
      "  void operator()()\n"
      "  {\n");
   os << tpl.Format(replacements);
   // TODO: remove child_id
  std::set<IntPair> written_blocks;
  GenCrbaLink(os, written_blocks, model_.child_id(NO_PARENT, 0u));
  os << "//H.triangularView<Eigen::Lower>() = \n"
        "  }\n"
        "};\n";
  return os.str();
}

RobotBuilder::Status RobotBuilderP::Write() {
  if (!is_initialized_) {
    std::cerr
        << "ERROR: the robot has no link."
        << std::endl;
    return RobotBuilder::STATUS_FAILURE;
  }

  // check name and libname
  if (name_.empty()) {
    std::cerr
        << "ERROR: the robot name has not been provided."
        << std::endl;
    return RobotBuilder::STATUS_FAILURE;
  }
  if (libname_.empty()) {
    std::cerr
        << "ERROR: the robot library name has not been provided."
        << std::endl;
    return RobotBuilder::STATUS_FAILURE;
  }
  // except for this line where the model is changed, the Write() would be const
  if (!model_.AssignDofIndexes()) {
    std::cerr
        << "ERROR: could not assign dof indexes to the joints. "
        << "Try adding the joint variables in another order or set the dof"
        << " indexes explicitly." << std::endl;
    return RobotBuilder::STATUS_FAILURE;
  }

  // create the directory (and its parents if necessary)
  boost::filesystem::create_directories(directory_);
#ifdef _MSC_VER
  // by default, MC VS prints numbers in scentific format with 3 digits for
  // the exponent. Eg: 3.2110E-005. We only want two digits like other
  // platforms do. Eg: 3.2110E-05.
  unsigned int old_exponent_format = _set_output_format(_TWO_DIGIT_EXPONENT);
#endif

  // fill the replacements we already know
  ReplMap repl;
  std::string libname_uc(libname_);
  std::transform(libname_.begin(), libname_.end(), libname_uc.begin(),
                 ::toupper);
  repl["LIBRARY_NAME"] = libname_uc;
  std::stringstream export_symbol;
  export_symbol << libname_ << "_EXPORTS";
  repl["EXPORT_SYMBOL"] = export_symbol.str();
  repl["ROBOT_CLASS_NAME"] = name_;
  repl["ROBOT_NAME"] = name_;
  repl["LICENSE"] = license_;
  repl["ROBOT_NB_DOF"] = ::to_string(model_.max_dof_index() + 1);
  repl["ROBOT_NB_REAL_DOF"] = ::to_string(model_.nb_dof());
  repl["ROBOT_NB_BODIES"] = ::to_string(model_.nb_links());

  // add the links to the temporary streams
  TmpStreams streams;
  for (int i = 0; i != model_.nb_links(); ++i)
    WriteLink(i, repl, streams);

  // complete the replacements map
  repl["nodeid_enum_definition"] = streams.nodeid_enum_definition.str();
  repl["node_type_definitions"] = streams.node_type_definitions.str();
  repl["nodes_type_list"] = streams.nodes_type_list.str();
  repl["map_node_id_to_rtnode"] = streams.map_node_id_to_rtnode.str();
  repl["crba"] = GenCrba(repl);
  for (int i = 0; i<MAX_NB_CHILDREN_PER_NODE; ++i) {
    std::stringstream key;
    key << "root_child" << i << "_id";
    repl[key.str()] = ::to_string(model_.child_id(NO_PARENT, i));
  }

  // init.cc
  repl["init_nodes"] = streams.init_nodes.str();
  repl["init_inertias"] = streams.init_inertias.str();

  // generate files from template and replacements
  const std::string config_hh_templ(::config_hh, ::config_hh_len);
  WriteTemplate("config.hh", config_hh_templ, repl);

  const std::string init_hh_templ(::init_hh, ::init_hh_len);
  WriteTemplate(name_ + ".hh", init_hh_templ, repl);

  const std::string init_cc_templ(::init_cc, ::init_cc_len);
  WriteTemplate(name_ + ".cc", init_cc_templ, repl);
#ifdef _MSC_VER
  // restore orginal exponent formatting (warning: this may never be reached
  // if an exception is thrown in between).
  _set_output_format(old_exponent_format);
#endif
  return RobotBuilder::STATUS_SUCCESS;
}

}
