open Printf
open Lwt
open Camlp4.PreCast
module MoveGram = MakeGram(Lexer)

let gdie = ref false
let gcolor = ref (1.0, 1.0, 1.0)
let gangle = ref 0.0
let gdragging = ref false 
let glsize = ref (500.0, 500.0)
let gquaternion = ref (0.0, 0.0, 0.0, 0.0)
let grotmtx = ref [|[|1.;0.;0.;0.|];[|0.;1.;0.;0.|];[|0.;0.;1.;0.|];[|0.;0.;0.;1.|]|]
let gnewrotmtx = ref [|[|1.;0.;0.;0.|];[|0.;1.;0.;0.|];[|0.;0.;1.;0.|];[|0.;0.;0.;1.|]|]
let gzoom = ref 1.0
let gfov = ref (100.0, 100.0)
let gpanning = ref false 
let gpanstart = ref (0.0, 0.0)
let gpan = ref (0.0, 0.0)
let goffset = ref (0.0, 0.0, 0.0)
let gnewoffset = ref (0.0, 0.0, 0.0)
let gneedle = ref None
let gcartridge = ref None
let gposlabel = ref None
let gvarlabel = ref None
let gpos = ref [| 0; 1 |]
let genv = ref []
let pi = acos(-1.0)

let read_stl fname = 
	let ic = BatFile.open_in fname in
	for i=0 to 79 do
		ignore(BatIO.read_byte ic); 
	done;
	let ntri = BatIO.read_i32 ic in
	let trilist = ref [] in
	for i=0 to ntri-1 do
		let n = (BatIO.read_float ic),(BatIO.read_float ic),(BatIO.read_float ic) in
		let a = (BatIO.read_float ic),(BatIO.read_float ic),(BatIO.read_float ic) in
		let b = (BatIO.read_float ic),(BatIO.read_float ic),(BatIO.read_float ic) in
		let c = (BatIO.read_float ic),(BatIO.read_float ic),(BatIO.read_float ic) in
		trilist := (n,a,b,c) :: !trilist; 
		ignore(BatIO.read_i16 ic); 
	done; 
	printf "%d triangles!\n%!" ntri; 
	let lst = GlList.create `compile in 
	GlDraw.begins `triangles;
	List.iter (fun (n,a,b,c) -> 
		GlDraw.normal3 n;
		GlDraw.vertex3 a;
		GlDraw.vertex3 b; 
		GlDraw.vertex3 c
	) !trilist; 
	GlDraw.ends ();
	GlList.ends ();
	lst
	;;
	
let normalizeQuaternion (qx,qy,qz,qw) = 
	let l = sqrt(qx*.qx +. qy*.qy +. qz*.qz +. qw*.qw) in
	if l > 0.0001 then 
	qx /. l, qy /. l, qz /. l, qw /. l 
	else qx,qy,qz,qw
;;
(* http://stackoverflow.com/questions/1556260/convert-quaternion-rotation-to-rotation-matrix *)
let quaternionToMatrix (qx,qy,qz,qw) = [|
	[|1.0 -. 2.0*.qy*.qy -. 2.0*.qz*.qz; 2.0*.qx*.qy -. 2.0*.qz*.qw; 2.0*.qx*.qz +. 2.0*.qy*.qw; 0.0|];
	[|2.0*.qx*.qy +. 2.0*.qz*.qw; 1.0 -. 2.0*.qx*.qx -. 2.0*.qz*.qz; 2.0*.qy*.qz -. 2.0*.qx*.qw; 0.0|];
	[|2.0*.qx*.qz -. 2.0*.qy*.qw; 2.0*.qy*.qz +. 2.0*.qx*.qw; 1.0 -. 2.0*.qx*.qx -. 2.0*.qy*.qy; 0.0|];
	[|0.0; 0.0; 0.0; 1.0|] |]
;;
let qinv (qx,qy,qz,qw) = (qx, qy, qz, -1.0 *. qz) ;;

let socksock host portno rx = 
	let sock = Lwt_unix.socket Unix.PF_INET Unix.SOCK_DGRAM 0 in
	let haddr = if rx 
		then Unix.inet_addr_any
		else (Unix.gethostbyname host).Unix.h_addr_list.(0) in
	let addr =  Unix.ADDR_INET (haddr, portno) in
	Lwt_unix.setsockopt sock Unix.SO_REUSEADDR true ; 
	let _ = if rx then (
		try Lwt_unix.bind sock addr
		with Unix.Unix_error (f,_,_) -> printf "bind error: %s\n%!" (Unix.error_message f) 
	) else (
		try Lwt.ignore_result(Lwt_unix.connect sock addr)
		with Unix.Unix_error (f,_,_) -> printf "connect error: %s\n%!" (Unix.error_message f) 
	) in
	sock,addr ;;
	
let sendto sock msg = 
	let s,a = sock in
	Lwt_unix.sendto s msg 0 (String.length msg) [] a ;;
	
	(* clean out an input buffer on a socket *)
let flushsock sock = 
	let buffer = String.create 512 in
	let s,_ = sock in
	let fd = Lwt_unix.unix_file_descr s in
	let rec read () = 
		let readable = Unix.select [fd] [] [] 0.0 <> ([], [], []) in
		if readable then (
			ignore(Unix.read fd buffer 0 512) ; (* this is blocking! *)
			read ()
		)
	in
	read () ;;

	(* get a message from a socket. always get some response. *)
let recvfrom sock buffer = 
	let s,_ = sock in
	let fd = Lwt_unix.unix_file_descr s in
	let rec recv () = 
		let readable = Unix.select [fd] [] [] 0.0 <> ([], [], []) in
		if readable then (
			Lwt_unix.read s buffer 0 512
		) else (
			Lwt_unix.sleep 0.03 >>= (fun _ -> recv () )
		)
	in
	recv () ;;
	(* always gets some response.. *)
	
	(* this just waits for the status, and passes the result as a thread. *)
let wait_status sock = 
	let buffer = String.create 512 in
	recvfrom sock buffer >>=
	(fun len -> 
		let ss = String.sub buffer 0 len in
		(* printf "status: %s %!" ss ;*)
		return ss 
	) ;;
	
let g_controlSock = socksock "localhost" 4593 false (* client, TX *)
let g_statusSock = socksock "localhost" 4594 true (*server, RX *)
let g_positionSock = socksock "localhost" 4595 true (* server, RX *)

let g_sockets = [g_statusSock; g_positionSock] 
	
	(* close all server sockets *)
let close_sockets () = 
	let closesock sock =
		try Unix.shutdown sock Unix.SHUTDOWN_ALL; 
		with Unix.Unix_error(f,_,_) -> printf "could not shut socket down, %s\n%!" (Unix.error_message f); 
		try Unix.close sock ; 
		with Unix.Unix_error(f,_,_) -> printf "could not close socket, %s\n%!" (Unix.error_message f); 
	in
	List.iter closesock (List.map Lwt_unix.unix_file_descr 
		(List.map fst g_sockets))
	;;

	
let foi = float_of_int ;; 
let iof = int_of_float ;;
let rotscl =  1.0 /. (16.0 *. 200.0) ;; (* 16 microstep, 200 step/rev motor *)
let linscl = 0.012192 /. 16.0 ;; (* 0.00048in/step, 16 microstep *)
let position_to_real ar = 
	Array.mapi (fun i d ->
		match i with 
		| 0 -> (foi d) *. rotscl
		| 2 -> (foi d) *. rotscl
		| _ -> (foi d) *. linscl
	) ar;;
let real_to_position ar = 
	Array.mapi (fun i d ->
		match i with 
		| 0 -> iof (d /. rotscl)
		| 2 -> iof (d /. rotscl)
		| _ -> iof (d /. linscl)
	) ar;;
let axislabels = ["needle_r";
					"needle_z";
					"brake_r";
					"brake_z";
					"gantry_x";
					"gantry_y";
					"gantry_z" ] ;;
let axis_to_num ax = 
	let j = ref (-1) in 
	List.iteri (fun i s -> if (String.compare ax s) == 0 then j := i) axislabels ; 
	!j ;;
	
	(* get current robot position from parprt .. *)
let read_position () = 
	let buffer = String.create 512 in
	let rec recv len = 
		if len > 0 then (
			let ss = String.sub buffer 0 len in 
			let sp = try Some (Pcre.extract 
				~pat:"([-\d]+) ([-\d]+) ([-\d]+) ([-\d]+) ([-\d]+) ([-\d]+) ([-\d]+)" ss)
				with Not_found -> None
			in
			(match sp with 
				| Some sp -> 
					gpos := Array.of_list (List.map int_of_string (List.tl (Array.to_list sp))); 
				| _ -> () ); 
			let ps = List.fold_left2 (fun s l v -> 
						s ^ l ^ " : " ^ (string_of_float v) ^ "\n"
					) "" axislabels (Array.to_list (position_to_real !gpos)) in
			(match !gposlabel with 
				| Some l -> l#set_label ps
				| _ -> ()); 
		) ; 
		if !gdie then return 0 else (
		recvfrom g_positionSock buffer >>= 
			(fun l -> recv l)
		)
	in
	recv 0 ;;

type t = 
		| Segment of string * t list * t option 
		| Move of bool * string * t * t
		| Save of string * t 
		| Call of string
		| Cond of t * t list
		| Binop of t * (float -> float -> float) * t
		| Cmp of t * (float -> float -> bool) * t
		| Var of string
		| Axis of string
		| Const of float
		
let fos = float_of_string 
	
let expression = MoveGram.Entry.mk "expression";;

EXTEND MoveGram
	GLOBAL: expression;

	expression: 
	[ "segment"
	[ `LIDENT s; "("; l = LIST1 expression SEP ";"; ")"; ";"; e2 = OPT expression ->
			Segment(s, l, e2) ]
	| "subexpr"
	[ "abs"; `LIDENT ax; pos = SELF; vel = SELF -> 
			Move(true, ax, pos, vel)
	| "rel"; `LIDENT ax; pos = SELF; vel = SELF -> 
			Move(false, ax, pos, vel) 
	| "call"; `LIDENT n -> Call(n)  (* not needed ATM! *)
	| `UIDENT n; "="; e1 = SELF -> Save(n, e1) ]
	| "controlstruct"
	[ "if"; e1 = SELF; "("; sub = LIST1 expression SEP ";"; ")" -> 
			Cond(e1, sub) ]
	| "plus"
	[ e1 = SELF; "+"; e2 = SELF -> Binop(e1, ( +. ), e2)
	| e1 = SELF; "-"; e2 = SELF -> Binop(e1, ( -. ), e2)]
	| "times"
	[ e1 = SELF; "*"; e2 = SELF -> Binop(e1, ( *. ), e2)
	| e1 = SELF; "/"; e2 = SELF -> Binop(e1, ( /. ), e2)]
	| "cmp"
	[ e1 = SELF; "<"; e2 = SELF -> Cmp(e1, ( < ), e2)
	| e1 = SELF; ">"; e2 = SELF -> Cmp(e1, ( > ), e2)
	| e1 = SELF; ">="; e2 = SELF -> Cmp(e1, ( >= ), e2)
	| e1 = SELF; "<="; e2 = SELF -> Cmp(e1, ( <= ), e2)
	| e1 = SELF; "<>"; e2 = SELF -> Cmp(e1, ( <> ), e2)
	| e1 = SELF; "=="; e2 = SELF -> Cmp(e1, ( == ), e2) ]
	| "simple"
	[ "-"; `FLOAT(f,_) -> Const(f *. (-1.0))
	| `FLOAT(f, _) -> Const(f)
	| `UIDENT n -> Var(n)
	| `LIDENT n -> Axis(n)
	| "("; e = expression; ")" -> e ]]; 
END;;

let parse_move s =
	MoveGram.parse_string expression (Loc.mk "<string>") s;;
	
	
	(* check if called segment exists *)
	(* returns true if found (= no error) *)
let rec segment_check arg called = 
	match arg with 
	| Segment(name, sub, more) -> 
		if (String.compare name called) == 0 then true
		else (if List.exists (fun a -> segment_check a called) sub then true
			else (match more with 
				| Some n -> segment_check n called
				| _ -> false))
	| _ -> false
	;;
	(* check that axes names make sense *)
	(* returns true on error *)
let rec axes_check arg = 
	let axis_sub name = 
		let j = axis_to_num name in
		if j < 0 then(
			printf "Error: axis label %s does not exist!\n%!" name; 
			true 
		)else false
	in
	match arg with 
	| Segment(name, axes, more) ->
		if List.exists (fun a -> axes_check a) axes then true else
		(match more with 
		| Some n -> axes_check n
		| _ -> false)
	| Move(_, name, _, _) -> axis_sub name
	| Axis(name) -> axis_sub name
	| _ -> false ;;
	
	(* check axes and call names *)
	(* returns true on error *)
let code_check arg = 
	if not (axes_check arg) then (
		let rec called b = 
			match b with
			| Segment(_, sub, more) -> 
				if List.exists called sub then true else 
				(match more with 
				| Some n -> called n
				| _ -> false)
			| Call(a) -> 
				if not (segment_check arg a) then(
					printf "Error: call %s does not exist!\n%!" a; 
					true
				) else false
			| _ -> false
		in
		called arg
	) else false ;;
	
	(* check that all the variables are instantiated before running a segment *)
	(* returns true if there is a variable that is not set *)
let var_check arg env = 
	printf "var_check\n%!"; 
	let vars = ref [] in
	(* recurs only applies with segments (linked-list) *)
	let rec var_collect a recurs = 
		match a with 
		| Segment(_, sub, more) -> 
			List.iter(fun b -> var_collect b true) sub ;
			if recurs then 
				(match more with 
				| Some n -> var_collect n true
				| _ -> ()) else ()
		| Move(_, _, e1, e2) -> 
			var_collect e1 false; var_collect e2 false
		| Save(n, e1) -> 
			var_collect e1 false
		| Cond(e1, sub) ->
			List.iter(fun b -> var_collect b true) sub ;
			var_collect e1 false
		| Binop(e1, _, e2) -> 
			var_collect e1 false; var_collect e2 false
		| Cmp(e1, _, e2) -> 
			var_collect e1 false; var_collect e2 false
		| Var(n) -> vars := n :: ! vars; 
		| _ -> ()
	in
	var_collect arg false; 
	List.exists (fun v -> 
		let set = List.mem_assoc v env in
		if not set then (
			printf "Error: variable %s not set!\n%!" v; 
			true
		) else false
	) !vars ;;
		
let build_cmd pos vel = 
	(List.fold_left2 (fun s p v -> 
		s ^ (string_of_int p) ^ " " ^ (string_of_int v) ^ " "
	) "m " 
	(Array.to_list (real_to_position pos))
	(Array.to_list (real_to_position vel))) ^ "\n" ;;
	
let run_cmd pos vel = 
	let cmd = build_cmd pos vel in
	printf " command: %s %!" cmd ; 
	sendto g_controlSock cmd
	>>= (fun _ -> 
		wait_status g_statusSock
	) >>= (fun s -> 
		let b = Pcre.pmatch ~pat:"done" s in
		if b then( printf "move done.\n%!"; return true )else( return false )
	) >>= (fun b -> 
		if b then Lwt_unix.sleep 0.1 (* make sure position is updated *)
		else return ()
	) >>= (fun _ -> return true)
	;;
	
let rec eval arg arg2 callfn pos vel = 
	let rec itersub d npos nvel = 
		match d with 
		| [] -> return (true,0.0)
		| hd :: tl -> (eval hd arg2 (fun _ -> true) npos nvel) 
			>>= (fun (a,b) -> 
				if a then itersub tl npos nvel
				else return (a,b))
	in
	match arg with
	| Segment(name, sub, more) -> 
		if callfn name then ( (* hence, only top-level calls possible *)
			printf "Executing %s (%d subs)\n%!" name (List.length sub); 
			let npos = [|0.0;0.0;0.0;0.0;0.0;0.0;0.0|] in
			let nvel = [|0.0;0.0;0.0;0.0;0.0;0.0;0.0|] in
			itersub sub npos nvel
			>>= (fun (a,b) -> 
				if a then (
					run_cmd npos nvel >>= (fun a -> 
						return (a, 0.0))
				)else return (false, 0.0) )
			>>= (fun (a,b) -> 
				if a then 
					(match more with
					| Some n -> eval n arg2 callfn pos vel
					| _ -> return (true, b))
				else return (false, b))
		) else return (true, 0.0)
	| Move(abs, name, pp, vv) -> 
		printf "Move %s\n" name;
		let curpos = position_to_real !gpos in
		let ax = axis_to_num name in
		let p = ref 0.0 in
		eval pp arg2 callfn pos vel 
		>>= (fun (a,b) -> 
			if a then (
				p := b; 
				eval vv arg2 callfn pos vel
			) else return (false, 0.0)
		) >>= (fun (a,v) -> 
			if a then (
				printf "axis %s (%d) %f %f\n%!" name ax !p v; 
				if abs then (
					pos.(ax) <- !p; 
					vel.(ax) <- v; 
				) else (
					pos.(ax) <- !p +. curpos.(ax); 
					vel.(ax) <- v; 
				) ; 
				return (true, 0.0)
			) else return (false, 0.0)
		)
	| Save(var, e1) -> 
		eval e1 arg2 callfn pos vel 
		>>= (fun (a,b) -> 
			genv := List.remove_assoc var !genv; 
			genv := (var, b) :: !genv; 
			(* update the GUI too! *)
			let keys,vals = List.split !genv in
			let txts = List.map2 (fun n v -> 
				n ^ " : " ^ (string_of_float v)) keys vals in
			
			let ps = List.fold_left (fun s l -> s ^ l ^ "\n"
					) "" (List.sort compare txts) in
			(match !gvarlabel with 
				| Some l -> l#set_label ps
				| _ -> ()); 
			return (true, b)
		)
	| Call(name) -> 
		(* search for the segment with this name and eval it *)
		(* no recursion! *)
		let callmask st = 
			if (String.compare name st) == 0 then true else false in
		eval arg2 arg2 callmask pos vel
	| Cond(e1, sub) -> 
		eval e1 arg2 callfn pos vel 
		>>= (fun (a,b) -> 
			if a then (
				itersub sub pos vel
			) else return (true, 0.0) )
	| Binop(e1, f, e2) -> 
		let v1 = ref 0.0 in
		eval e1 arg2 callfn pos vel 
		>>= (fun (a,b) -> 
			v1 := b; 
			eval e2 arg2 callfn pos vel
		) >>= (fun (a,v2) -> 
			return (true, (f !v1 v2))
		)
	| Cmp(e1, f, e2) -> 
		let v1 = ref 0.0 in
		eval e1 arg2 callfn pos vel 
		>>= (fun (a,b) -> 
			v1 := b; 
			eval e2 arg2 callfn pos vel
		) >>= (fun (a,v2) -> 
			let b = f !v1 v2 in
			return (b, 0.0)
		)
	| Var(name) -> 
		return (true, List.assoc name !genv)
	| Axis(name) -> 
		printf "Axis %s referenced.\n" name; 
		let curpos = position_to_real !gpos in
		let ax = axis_to_num name in
		return (true, curpos.(ax))
	| Const(f) -> return (true, f)
	;;

(* make buttons for the top-level segments. *)
let rec make_buttons arg box1 mvb = 
	match arg with 
	| Segment(name, sub, more) as me -> 
		let button = GButton.button ~label:name ~packing:box1#add () in
		mvb := button#coerce :: !mvb; 
		ignore(button#connect#clicked ~callback:(fun () -> 
			(* since variable binding is at runtime, must check just before executing *)
			if not (var_check arg !genv ) then (
			ignore_result(eval me arg 
				(fun n -> (String.compare n name) == 0) (* to prevent eval following movements *)
				[| |] [| |] )))); 
		(match more with 
		| Some n -> make_buttons n box1 mvb; 
		| _ -> ())
	| _ -> ()

let main () = (
	GMain.init();
	Lwt_glib.install ();
	
	let dextroy () = 
		gdie := true; 
		close_sockets (); 
		GMain.Main.quit (); 
		GMain.quit (); () in
	
	let w = GWindow.window () in
	let vbox1 = GPack.vbox ~packing:w#add () in
	let menubar = GMenu.menu_bar ~packing:vbox1#pack () in
	let menu_factory = new GMenu.factory menubar in
	let accel_group = menu_factory#accel_group in
	let file_menu = menu_factory#add_submenu "File" in
	let file_menu_factory = new GMenu.factory file_menu ~accel_group in
	
	let hbox = GPack.hbox ~packing:vbox1#add () in
	let notebook = GPack.notebook ~packing:hbox#add () in
	
	w#connect#destroy ~callback:dextroy; 
	let area = GlGtk.area [`RGBA;`DEPTH_SIZE 1;`DOUBLEBUFFER]
		~width:500 ~height:600 ~packing:(fun w -> ignore (notebook#append_page w)) () in
	area#connect#realize ~callback:(fun () ->
		let light_ambient = 0.0, 0.0, 0.0, 1.0
		and light_diffuse = 1.0, 1.0, 1.0, 1.0
		and light_specular = 1.0, 1.0, 1.0, 1.0
		(*  light_position is NOT default value	*)
		and light_position = 0.0, 1.0, 1.0, 0.0
		in
		GlLight.light ~num:0 (`ambient light_ambient);
		GlLight.light ~num:0 (`diffuse light_diffuse);
		GlLight.light ~num:0 (`specular light_specular);
		GlLight.light ~num:0 (`position light_position);
		
		GlFunc.depth_func `less;
		List.iter Gl.enable [`lighting; `light0; `depth_test; `color_material];
		(* load the STL into a display list (faster?) *)
		gneedle := Some (read_stl "/home/tlh24/sewing_machine/solidworks/inserter2_needlebrakeplate.STL");
		gcartridge := Some (read_stl "/home/tlh24/sewing_machine/solidworks/cartridge_3.STL");
		);
	let make_projection () = 
		GlMat.mode `projection;
		GlMat.load_identity ();
		let scl = 100.0 /. !gzoom in
		let ncl = -1.0 *. scl in
		let w,h = !glsize in
		if w <= h then (
			GlMat.ortho ~x:(ncl,scl) ~z:(-100.0,100.0)
			~y:(ncl *. h /. w, scl *. h /. w); 
			gfov := 2.0 *. scl, 2.0 *. scl *. h /. w; 
		) else (
			GlMat.ortho ~y:(ncl,scl) ~z:(-100.0,100.0)
			~x:(ncl *. w /. h, scl *. w /. h);
			gfov := 2.0 *. scl *. w /. h, 2.0 *. scl; 
		); 
		GlMat.mode `modelview
	in
	area#connect#reshape ~callback:(fun ~width:w ~height:h -> 
		GlDraw.viewport ~x:0 ~y:0 ~w ~h;
		glsize := ((float_of_int w),(float_of_int h)); 
		make_projection ()
		); 
	let draw () = 
		if not !gdie then (
		let k = try area#make_current (); true with _ -> false in
		if k then (
			GlClear.clear [`color; `depth];
			Gl.flush();
			GlMat.load_identity ();
			
			(* calculate the forward rotation matrix. (rotate model) *)
			GlMat.push ();
			GlMat.load (GlMat.of_array (quaternionToMatrix !gquaternion));
			GlMat.mult (GlMat.of_array !grotmtx); 
			gnewrotmtx := GlMat.to_array (GlMat.get_matrix `modelview_matrix); 
			GlMat.pop (); 
			
			(* and the inverse rotation matrix (rotate screen plane -- just the transpose) *)
			GlMat.push ();
			GlMat.translate3 ((fst !gpan), (snd !gpan), 0.0) ; 
			let t = GlMat.to_array (GlMat.get_matrix `modelview_matrix) in
			GlMat.load_transpose (GlMat.of_array !gnewrotmtx); 
			GlMat.mult (GlMat.of_array t); (* this should rotate the pan .. now accumulate *)
			let tr = GlMat.to_array (GlMat.get_matrix `modelview_matrix) in
			let tx,ty,tz = tr.(3).(0), tr.(3).(1), tr.(3).(2) in
			let ox,oy,oz = !goffset in
			gnewoffset := tx+.ox, ty+.oy, tz+.oz; 
			GlMat.pop (); 
			
			GlMat.push (); 
			GlMat.load (GlMat.of_array !gnewrotmtx);
			GlMat.translate3 !gnewoffset; (* translate then rotate *)
			
			let dolist b = match b with 
			| Some a -> GlList.call a
			| _ -> () in
			
			GlDraw.color (1.0,1.0,0.0);
			GlMat.push ();
			GlMat.scale3 (1.0, 1.0, -1.0); 
			GlMat.translate3 (6.5, 85.0, 20.96);
			GlMat.rotate ~x:1.0 ~angle:270.0 ();
			GlMat.rotate ~y:1.0 ~angle:90.0 ();
			dolist !gneedle; 
			GlMat.pop (); 
			
			GlDraw.color (0.0,1.0,1.0);
			GlMat.push ();
			GlMat.translate3 (0.0, 0.0, 0.0); 
			GlMat.rotate ~y:1.0 ~angle:270.0 ();
			dolist !gcartridge;
			GlMat.pop (); 
			
			(* draw a little cube at the origin *)
			GlDraw.color (1.0,0.0,0.0);
			GlDraw.begins `quads; 
			let face permute = 
				let hface s = 
					GlDraw.normal3 (permute (0.0, 0.0, s));
					GlDraw.vertex3 (permute (1.0, 1.0, s));
					GlDraw.vertex3 (permute (1.0, -1.0, s));
					GlDraw.vertex3 (permute (-1.0, -1.0, s));
					GlDraw.vertex3 (permute (-1.0, 1.0, s));
				in
				hface 1.0; 
				hface (-1.0); 
			in
			face (fun (x,y,z) -> (x,y,z));
			face (fun (x,y,z) -> (y,z,x)); 
			face (fun (x,y,z) -> (z,x,y)); 
			GlDraw.ends (); 
			
			GlMat.pop (); 
			Gl.flush ();
			area#swap_buffers ();
			true
		) else false ) else false
	in
	area#connect#display ~callback:(fun () -> draw(); ());
	let mapToGl (x,y) = 
		1.0 -. 2.0 *. x /. (fst !glsize), 
		2.0 *. y /. (snd !glsize) -. 1.0 in
	let mapToArcball (x2,y2) = 
		let length = x2 *. x2 +. y2 *. y2 in
		if length > 1.0 then (
			let norm = 1.0 /. sqrt(length) in
			x2 /. norm, y2 /. norm, 0.0
		) else x2, y2, sqrt (1.0 -. length) 
	in
	area#event#add [`BUTTON_PRESS];
	area#event#connect#button_press ~callback:(fun ev ->
		area#make_current ();
		if GdkEvent.Button.button ev == 1 then (
			gpanstart := mapToGl (GdkEvent.Button.x ev, GdkEvent.Button.y ev);
			gdragging := true;
		); 
		if GdkEvent.Button.button ev == 3 then (
			gpanstart := mapToGl (GdkEvent.Button.x ev, GdkEvent.Button.y ev);
			gpanning := true; 
		); 
		true); 
	area#event#add [`BUTTON_RELEASE];
	area#event#connect#button_release ~callback:(fun ev ->
		area#make_current ();
		if GdkEvent.Button.button ev == 1 then (
			gdragging := false; 
			grotmtx := !gnewrotmtx;
			gquaternion := (0.0, 0.0, 0.0, 0.0); 
		); 
		if GdkEvent.Button.button ev == 3 then (
			gpanning := false; 
			goffset := !gnewoffset; 
			gpan := 0.0, 0.0;
		); 
		true); 
	area#event#add [`POINTER_MOTION];
	area#event#connect#motion_notify ~callback:(fun ev ->
		if !gdragging then (
			area#make_current ();
			let x1,y1,z1 = (0.0, 0.0, 1.0) in
			let x,y = mapToGl (GdkEvent.Motion.x ev, GdkEvent.Motion.y ev) in
			let ox,oy = !gpanstart in
			let x2,y2,z2 = mapToArcball (x -. ox, y -. oy) in
			(* cross product of the two vectors *)
			let cx,cy,cz = y1 *. z2 -. z1 *. y2, 
								z1 *. x2 -. x1 *. z2, 
								x1 *. y2 -. y1 *. x2 in
			let dot (a,b,c) (d,e,f) = a *. d +. b *. e +. c *. f in
			let l = cx *. cx +. cy *. cy +. cz *. cz in
			let qx,qy,qz,qw = if l > 0.0001 then (
				cx, cy, cz, (dot (x1,y1,z1) (x2,y2,z2))
			) else (
				0.0, 0.0, 0.0, 0.0
			) in
			gquaternion := normalizeQuaternion (qx,qy,qz,qw); 
		);
		if !gpanning then (
			area#make_current ();
			let x,y = mapToGl (GdkEvent.Motion.x ev, GdkEvent.Motion.y ev) in
			let ox,oy = !gpanstart in
			let fx,fy = !gfov in
			gpan := ((ox -. x) *. fx *. 0.5, (oy -. y) *. fy  *. 0.5 ) ; 
		); 
		true); 
	area#event#add [`SCROLL];
	area#event#connect#scroll ~callback:(fun ev -> 
		match (GdkEvent.Scroll.direction ev) with
		| `DOWN -> gzoom := !gzoom *. 1.1; make_projection (); true
		| `UP -> gzoom := !gzoom *. 0.9; make_projection (); true
		| _ -> false
		); 
	(* vsync must be disabled! *)
	GMain.Timeout.add ~ms:33 ~callback:(fun () -> draw ()); 
	let vbox = GPack.vbox ~border_width:3 ~spacing:4 ~packing:hbox#pack () in
	(* location text box. *)
	gposlabel := Some (GMisc.label ~text:"" ~packing:vbox#pack () );
	gvarlabel := Some (GMisc.label ~text:"No variables yet" ~packing:vbox#pack () );
	
	(* source view *)
	let scrolled_win = GBin.scrolled_window
		~hpolicy: `AUTOMATIC ~vpolicy: `AUTOMATIC
		~packing:(fun w -> ignore (notebook#append_page w)) () in
	let source_view =
		GSourceView2.source_view
			~auto_indent:true
			~insert_spaces_instead_of_tabs:false ~tab_width:2
			~show_line_numbers:true
			~right_margin_position:30 ~show_right_margin:true
			~smart_home_end:`ALWAYS
			~packing:scrolled_win#add ~height:500 ~width:500
			() in
	source_view#misc#modify_font_by_name "DejaVu Sans Mono 11";
	let text =
		let ic = open_in "movements.txt" in
		let size = in_channel_length ic in
		let buf = String.create size in
		really_input ic buf 0 size;
		close_in ic;
		buf
	in
	source_view#source_buffer#set_text text;
	source_view#source_buffer#set_highlight_syntax true;
	let language_manager = GSourceView2.source_language_manager ~default:true in
	let lang =
		(match language_manager#guess_language ~content_type:"text/x-ocaml" () with
			Some x -> x
		| None -> failwith (sprintf "no language for %s" "text/x-ocaml"))
	in
	source_view#source_buffer#set_language (Some lang);

	
	let movement_buttons = ref [] in
	let testb = GButton.button ~label:"compile" ~packing:vbox#pack () in
	testb#connect#clicked ~callback:(fun () -> 
		(* remove all the old buttons *)
		List.iter (fun b -> vbox#remove b) !movement_buttons; 
		let s = source_view#source_buffer#get_text () in
		let r = try Some (parse_move s) with e -> 
			Format.eprintf "error: %a@." Camlp4.ErrorHandler.print e;
			None in
		(match r with
			| Some k -> 
				if not (code_check k) then( 
					make_buttons k vbox movement_buttons
				) else ()
			| _ -> ()); 
	);
	
	file_menu_factory#add_item "Save" ~key:GdkKeysyms._S ~callback:(fun () -> 
		printf "saving movements.txt\n%!"; 
		let oc = open_out "movements.txt" in
		fprintf oc "%s" (source_view#source_buffer#get_text ()); 
		flush oc; 
		close_out_noerr oc) ; 
	file_menu_factory#add_item "Quit" ~callback:dextroy;
	
	w#add_accel_group accel_group;
	w#show (); 
	
	ignore(Lwt_unix.run (read_position () ) ); 
	
	GMain.main () ; 
	gdie := true; 
);;

let _ = main ()