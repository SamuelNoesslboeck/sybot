use alloc::collections::BTreeMap;

pub type Pins = BTreeMap<String, BTreeMap<String, u8>>;

pub fn get_pin(pins : &Pins, path : String) -> Option<u8> {
    let path_s : Vec<&str> = path.split('/').collect();

    if path_s.len() != 2 {
        panic!(" => Bad path format! (<group>/<name>) [{:?}]", path_s);
    }

    if let Some(group) = pins.get(path_s[0]) {
        if let Some(pin) = group.get(path_s[1]) {
            Some(*pin)
        } else {
            None
        }
    } else {
        None
    }
}
