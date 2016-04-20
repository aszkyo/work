function CodeDefine() { 
this.def = new Array();
this.def["rt_OneStep"] = {file: "ert_main_c.html",line:35,type:"fcn"};
this.def["main"] = {file: "ert_main_c.html",line:72,type:"fcn"};
this.def["APGS_B"] = {file: "APGS_c.html",line:33,type:"var"};
this.def["APGS_DW"] = {file: "APGS_c.html",line:36,type:"var"};
this.def["APGS_U"] = {file: "APGS_c.html",line:39,type:"var"};
this.def["APGS_M_"] = {file: "APGS_c.html",line:42,type:"var"};
this.def["APGS_M"] = {file: "APGS_c.html",line:43,type:"var"};
this.def["APGS_validate_parkng_space"] = {file: "APGS_c.html",line:56,type:"fcn"};
this.def["APGS_get_second_pos"] = {file: "APGS_c.html",line:181,type:"fcn"};
this.def["APGS_edge1_Det"] = {file: "APGS_c.html",line:239,type:"fcn"};
this.def["APGS_get_first_pos"] = {file: "APGS_c.html",line:414,type:"fcn"};
this.def["APGS_edge0_Det"] = {file: "APGS_c.html",line:441,type:"fcn"};
this.def["rt_roundd"] = {file: "APGS_c.html",line:575,type:"fcn"};
this.def["APGS_step"] = {file: "APGS_c.html",line:594,type:"fcn"};
this.def["APGS_initialize"] = {file: "APGS_c.html",line:1871,type:"fcn"};
this.def["APGS_terminate"] = {file: "APGS_c.html",line:2022,type:"fcn"};
this.def["B_APGS_T"] = {file: "APGS_h.html",line:51,type:"type"};
this.def["DW_APGS_T"] = {file: "APGS_h.html",line:103,type:"type"};
this.def["ExtU_APGS_T"] = {file: "APGS_h.html",line:123,type:"type"};
this.def["P_APGS_T"] = {file: "APGS_types_h.html",line:23,type:"type"};
this.def["RT_MODEL_APGS_T"] = {file: "APGS_types_h.html",line:26,type:"type"};
this.def["APGS_P"] = {file: "APGS_data_c.html",line:22,type:"var"};
this.def["int8_T"] = {file: "rtwtypes_h.html",line:51,type:"type"};
this.def["uint8_T"] = {file: "rtwtypes_h.html",line:52,type:"type"};
this.def["int16_T"] = {file: "rtwtypes_h.html",line:53,type:"type"};
this.def["uint16_T"] = {file: "rtwtypes_h.html",line:54,type:"type"};
this.def["int32_T"] = {file: "rtwtypes_h.html",line:55,type:"type"};
this.def["uint32_T"] = {file: "rtwtypes_h.html",line:56,type:"type"};
this.def["real32_T"] = {file: "rtwtypes_h.html",line:57,type:"type"};
this.def["real64_T"] = {file: "rtwtypes_h.html",line:58,type:"type"};
this.def["real_T"] = {file: "rtwtypes_h.html",line:64,type:"type"};
this.def["time_T"] = {file: "rtwtypes_h.html",line:65,type:"type"};
this.def["boolean_T"] = {file: "rtwtypes_h.html",line:66,type:"type"};
this.def["int_T"] = {file: "rtwtypes_h.html",line:67,type:"type"};
this.def["uint_T"] = {file: "rtwtypes_h.html",line:68,type:"type"};
this.def["ulong_T"] = {file: "rtwtypes_h.html",line:69,type:"type"};
this.def["char_T"] = {file: "rtwtypes_h.html",line:70,type:"type"};
this.def["uchar_T"] = {file: "rtwtypes_h.html",line:71,type:"type"};
this.def["byte_T"] = {file: "rtwtypes_h.html",line:72,type:"type"};
this.def["creal32_T"] = {file: "rtwtypes_h.html",line:82,type:"type"};
this.def["creal64_T"] = {file: "rtwtypes_h.html",line:87,type:"type"};
this.def["creal_T"] = {file: "rtwtypes_h.html",line:92,type:"type"};
this.def["cint8_T"] = {file: "rtwtypes_h.html",line:99,type:"type"};
this.def["cuint8_T"] = {file: "rtwtypes_h.html",line:106,type:"type"};
this.def["cint16_T"] = {file: "rtwtypes_h.html",line:113,type:"type"};
this.def["cuint16_T"] = {file: "rtwtypes_h.html",line:120,type:"type"};
this.def["cint32_T"] = {file: "rtwtypes_h.html",line:127,type:"type"};
this.def["cuint32_T"] = {file: "rtwtypes_h.html",line:134,type:"type"};
this.def["pointer_T"] = {file: "rtwtypes_h.html",line:155,type:"type"};
this.def["ZCDirection"] = {file: "rtwtypes_h.html",line:166,type:"type"};
this.def["ZCSigState"] = {file: "rtwtypes_h.html",line:169,type:"type"};
this.def["ZCEventType"] = {file: "rtwtypes_h.html",line:182,type:"type"};
}
CodeDefine.instance = new CodeDefine();
function Html2SrcLink() {
	this.html2SrcPath = new Array;
	this.html2Root = new Array;
	this.html2SrcPath["ert_main_c.html"] = "../ert_main.c";
	this.html2Root["ert_main_c.html"] = "ert_main_c.html";
	this.html2SrcPath["APGS_c.html"] = "../APGS.c";
	this.html2Root["APGS_c.html"] = "APGS_c.html";
	this.html2SrcPath["APGS_h.html"] = "../APGS.h";
	this.html2Root["APGS_h.html"] = "APGS_h.html";
	this.html2SrcPath["APGS_private_h.html"] = "../APGS_private.h";
	this.html2Root["APGS_private_h.html"] = "APGS_private_h.html";
	this.html2SrcPath["APGS_types_h.html"] = "../APGS_types.h";
	this.html2Root["APGS_types_h.html"] = "APGS_types_h.html";
	this.html2SrcPath["APGS_data_c.html"] = "../APGS_data.c";
	this.html2Root["APGS_data_c.html"] = "APGS_data_c.html";
	this.html2SrcPath["rtwtypes_h.html"] = "../rtwtypes.h";
	this.html2Root["rtwtypes_h.html"] = "rtwtypes_h.html";
	this.getLink2Src = function (htmlFileName) {
		 if (this.html2SrcPath[htmlFileName])
			 return this.html2SrcPath[htmlFileName];
		 else
			 return null;
	}
	this.getLinkFromRoot = function (htmlFileName) {
		 if (this.html2Root[htmlFileName])
			 return this.html2Root[htmlFileName];
		 else
			 return null;
	}
}
Html2SrcLink.instance = new Html2SrcLink();
var fileList = [
"ert_main_c.html","APGS_c.html","APGS_h.html","APGS_private_h.html","APGS_types_h.html","APGS_data_c.html","rtwtypes_h.html"];
