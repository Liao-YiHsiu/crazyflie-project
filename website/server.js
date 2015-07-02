var express = require("express");
var exphbs = require("express-handlebars");
var bodyParser = require("body-parser");
var app = express();

app.use(bodyParser());
app.engine(".hbs" , exphbs({
    extname: ".hbs",
    defaultLayout: "main"
}));
app.set("view engine", ".hbs");
app.use(express.static(__dirname + '/public'));
app.get("/" , function(req , res) {
    res.render("home");
});

app.get("/tech" , function(req , res) {
    res.render("tech");
});
app.get("/demo" , function(req , res) {
    res.render("demo");
});
app.get("/about-us" , function(req , res) {
    res.render("about-us");
});
app.get("/ref" , function(req , res) {
    res.render("ref");
});

app.listen( 5246  , function() {
    console.log("iAgent Server. 140.112.21.15:5246 ");

});
