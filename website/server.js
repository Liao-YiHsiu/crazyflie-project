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


app.listen( 5246  , function() {
    console.log("iAgent Server. 140.112.21.15:5246 ");

});
