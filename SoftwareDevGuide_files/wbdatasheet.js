if(!window.com) com = new Object();
if(!com.TI) com.TI = new Object();

com.TI.wbpanel = {
wbConfig : function (litNum) {
	  var tiLibraryStore = new com.TI.tiLibrary.tiLibraryStore();
      var documentLocale = tiLibraryStore.viewer_store.documentLocale;
	if($("#webenchControl").length <= 0 || documentLocale !== "en") {
		return;
	}
	var urlString = "https://webench.ti.com/wb5/InteractivePanels/power-part.jsp?p=";
	urlString += litNum;
	var xmlhttp;
	if (window.XMLHttpRequest)
		xmlhttp = new XMLHttpRequest();
	else
		xmlhttp = new ActiveXObject("Microsoft.XMLHTTP");
	xmlhttp.onreadystatechange = function() {
		if (xmlhttp.readyState === 4 && xmlhttp.status === 200 ) {
			$("#webenchControl").html( xmlhttp.responseText );
			eval($("#webenchControl script").text());
		}
	};
	xmlhttp.open("GET", urlString, true);
	xmlhttp.send();
},
openDesign : function(form, doc_self) {
	var url = "https://webench.ti.com/power-designer/switching-regulator/create/customize?base_pn="
			+ form.base_pn.value + "&VinMin=" + form.VinMin.value + "&VinMax="
			+ form.VinMax.value + "&O1V=" + form.O1V.value + "&O1I="
			+ form.O1I.value + "&AppType=" + form.AppType.value + "&op_TA="
			+ form.op_TA.value + "&lang_chosen=en_US&origin=online_datasheet" + "&optfactor=3";
	if(typeof(form.topology) != undefined) {
		url += "&topology=" + form.topology.value;
	}
	if(typeof(form.flavor) != undefined) {
		url += "&flavor=" + form.flavor.value;
	}
	window.open(url,"_blank");
	_tiAnalyticsTrack('Literature reader webench link', doc_self.litnum, doc_self.litchapter, doc_self.litsection, 'webench create design', 'Open Design');
},
printDesign : function(form, doc_self) {
	var url = "https://webench.ti.com/wb5/WBTablet/PartDesigner/quickview_pdf.jsp?base_pn="
			+ form.base_pn.value + "&VinMin=" + form.VinMin.value + "&VinMax="
			+ form.VinMax.value + "&O1V=" + form.O1V.value + "&O1I="
			+ form.O1I.value + "&AppType=" + form.AppType.value + "&op_TA="
			+ form.op_TA.value + "&optfactor=3&flag=false";
	 if(typeof(form.topology) != undefined) {
        url += "&topology=" + form.topology.value;
    }
    if(typeof(form.flavor) != undefined) {
        url += "&flavor=" + form.flavor.value;
    }
	window.open(url,"_blank");
	_tiAnalyticsTrack('Literature reader webench link', doc_self.litnum, doc_self.litchapter, doc_self.litsection, 'webench link click', 'Generate Report');
}
};