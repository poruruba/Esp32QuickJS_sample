/* example of JS module importing a C module */

import { fib } from "fib.js";
import * as math from "math.js";

setInterval(() =>{
  console.log("Hello World");
  console.log("fib(10)=", fib(10));
  console.log("add(1, 2)=", math.add(1, 2));
  console.log("sub(3, 4)=", math.sub(3, 4));
}, 1000);
