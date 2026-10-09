//! Bluetooth Low Energy HCI packets as exchanged with an STM32WB radio
//! co-processor (standard HCI plus ST's vendor-specific ACI commands and
//! events). Used by the SEPROXYHAL decoder, which carries them in its
//! `BLE_SEND` / `BLE_RECV_EVENT` packets.
//!
//! Command and vendor event names are those of the STM32WB BLE stack API.

use super::seph::hex_trunc;

/// HCI / ACI command opcodes (OGF << 10 | OCF) and names.
pub(crate) const COMMANDS: &[(u16, &str)] = &[
    (0x0406, "hci_disconnect"),
    (0x041d, "hci_read_remote_version_information"),
    (0x0c01, "hci_set_event_mask"),
    (0x0c03, "hci_reset"),
    (0x0c2d, "hci_read_transmit_power_level"),
    (0x0c31, "hci_set_controller_to_host_flow_control"),
    (0x0c33, "hci_host_buffer_size"),
    (0x0c35, "hci_host_number_of_completed_packets"),
    (0x1001, "hci_read_local_version_information"),
    (0x1002, "hci_read_local_supported_commands"),
    (0x1003, "hci_read_local_supported_features"),
    (0x1009, "hci_read_bd_addr"),
    (0x1405, "hci_read_rssi"),
    (0x2001, "hci_le_set_event_mask"),
    (0x2002, "hci_le_read_buffer_size"),
    (0x2003, "hci_le_read_local_supported_features"),
    (0x2005, "hci_le_set_random_address"),
    (0x2006, "hci_le_set_advertising_parameters"),
    (0x2007, "hci_le_read_advertising_channel_tx_power"),
    (0x2008, "hci_le_set_advertising_data"),
    (0x2009, "hci_le_set_scan_response_data"),
    (0x200a, "hci_le_set_advertise_enable"),
    (0x200b, "hci_le_set_scan_parameters"),
    (0x200c, "hci_le_set_scan_enable"),
    (0x200d, "hci_le_create_connection"),
    (0x200e, "hci_le_create_connection_cancel"),
    (0x200f, "hci_le_read_white_list_size"),
    (0x2010, "hci_le_clear_white_list"),
    (0x2011, "hci_le_add_device_to_white_list"),
    (0x2012, "hci_le_remove_device_from_white_list"),
    (0x2013, "hci_le_connection_update"),
    (0x2014, "hci_le_set_host_channel_classification"),
    (0x2015, "hci_le_read_channel_map"),
    (0x2016, "hci_le_read_remote_features"),
    (0x2017, "hci_le_encrypt"),
    (0x2018, "hci_le_rand"),
    (0x2019, "hci_le_start_encryption"),
    (0x201a, "hci_le_long_term_key_request_reply"),
    (0x201b, "hci_le_long_term_key_requested_negative_reply"),
    (0x201c, "hci_le_read_supported_states"),
    (0x201d, "hci_le_receiver_test"),
    (0x201e, "hci_le_transmitter_test"),
    (0x201f, "hci_le_test_end"),
    (0x2022, "hci_le_set_data_length"),
    (0x2023, "hci_le_read_suggested_default_data_length"),
    (0x2024, "hci_le_write_suggested_default_data_length"),
    (0x2025, "hci_le_read_local_p256_public_key"),
    (0x2026, "hci_le_generate_dhkey"),
    (0x2027, "hci_le_add_device_to_resolving_list"),
    (0x2028, "hci_le_remove_device_from_resolving_list"),
    (0x2029, "hci_le_clear_resolving_list"),
    (0x202a, "hci_le_read_resolving_list_size"),
    (0x202b, "hci_le_read_peer_resolvable_address"),
    (0x202c, "hci_le_read_local_resolvable_address"),
    (0x202d, "hci_le_set_address_resolution_enable"),
    (0x202e, "hci_le_set_resolvable_private_address_timeout"),
    (0x202f, "hci_le_read_maximum_data_length"),
    (0x2030, "hci_le_read_phy"),
    (0x2031, "hci_le_set_default_phy"),
    (0x2032, "hci_le_set_phy"),
    (0x2033, "hci_le_enhanced_receiver_test"),
    (0x2034, "hci_le_enhanced_transmitter_test"),
    (0x204b, "hci_le_read_transmit_power"),
    (0x204e, "hci_le_set_privacy_mode"),
    (0xfc00, "aci_hal_get_fw_build_number"),
    (0xfc0c, "aci_hal_write_config_data"),
    (0xfc0d, "aci_hal_read_config_data"),
    (0xfc0f, "aci_hal_set_tx_power_level"),
    (0xfc14, "aci_hal_le_tx_test_packet_number"),
    (0xfc15, "aci_hal_tone_start"),
    (0xfc16, "aci_hal_tone_stop"),
    (0xfc17, "aci_hal_get_link_status"),
    (0xfc18, "aci_hal_set_radio_activity_mask"),
    (0xfc19, "aci_hal_get_anchor_period"),
    (0xfc1a, "aci_hal_set_event_mask"),
    (0xfc1b, "aci_hal_set_smp_eng_config"),
    (0xfc1c, "aci_hal_get_pm_debug_info"),
    (0xfc30, "aci_hal_read_radio_reg"),
    (0xfc31, "aci_hal_write_radio_reg"),
    (0xfc32, "aci_hal_read_raw_rssi"),
    (0xfc33, "aci_hal_rx_start"),
    (0xfc34, "aci_hal_rx_stop"),
    (0xfc3b, "aci_hal_stack_reset"),
    (0xfc81, "aci_gap_set_non_discoverable"),
    (0xfc82, "aci_gap_set_limited_discoverable"),
    (0xfc83, "aci_gap_set_discoverable"),
    (0xfc84, "aci_gap_set_direct_connectable"),
    (0xfc85, "aci_gap_set_io_capability"),
    (0xfc86, "aci_gap_set_authentication_requirement"),
    (0xfc87, "aci_gap_set_authorization_requirement"),
    (0xfc88, "aci_gap_pass_key_resp"),
    (0xfc89, "aci_gap_authorization_resp"),
    (0xfc8a, "aci_gap_init"),
    (0xfc8b, "aci_gap_set_non_connectable"),
    (0xfc8c, "aci_gap_set_undirected_connectable"),
    (0xfc8d, "aci_gap_slave_security_req"),
    (0xfc8e, "aci_gap_update_adv_data"),
    (0xfc8f, "aci_gap_delete_ad_type"),
    (0xfc90, "aci_gap_get_security_level"),
    (0xfc91, "aci_gap_set_event_mask"),
    (0xfc92, "aci_gap_configure_whitelist"),
    (0xfc93, "aci_gap_terminate"),
    (0xfc94, "aci_gap_clear_security_db"),
    (0xfc95, "aci_gap_allow_rebond"),
    (0xfc96, "aci_gap_start_limited_discovery_proc"),
    (0xfc97, "aci_gap_start_general_discovery_proc"),
    (0xfc98, "aci_gap_start_name_discovery_proc"),
    (0xfc99, "aci_gap_start_auto_connection_establish_proc"),
    (0xfc9a, "aci_gap_start_general_connection_establish_proc"),
    (0xfc9b, "aci_gap_start_selective_connection_establish_proc"),
    (0xfc9c, "aci_gap_create_connection"),
    (0xfc9d, "aci_gap_terminate_gap_proc"),
    (0xfc9e, "aci_gap_start_connection_update"),
    (0xfc9f, "aci_gap_send_pairing_req"),
    (0xfca0, "aci_gap_resolve_private_addr"),
    (0xfca1, "aci_gap_set_broadcast_mode"),
    (0xfca2, "aci_gap_start_observation_proc"),
    (0xfca3, "aci_gap_get_bonded_devices"),
    (0xfca4, "aci_gap_is_device_bonded"),
    (0xfca5, "aci_gap_numeric_comparison_value_confirm_yesno"),
    (0xfca6, "aci_gap_passkey_input"),
    (0xfca7, "aci_gap_get_oob_data"),
    (0xfca8, "aci_gap_set_oob_data"),
    (0xfca9, "aci_gap_add_devices_to_resolving_list"),
    (0xfcaa, "aci_gap_remove_bonded_device"),
    (0xfd01, "aci_gatt_init"),
    (0xfd02, "aci_gatt_add_service"),
    (0xfd03, "aci_gatt_include_service"),
    (0xfd04, "aci_gatt_add_char"),
    (0xfd05, "aci_gatt_add_char_desc"),
    (0xfd06, "aci_gatt_update_char_value"),
    (0xfd07, "aci_gatt_del_char"),
    (0xfd08, "aci_gatt_del_service"),
    (0xfd09, "aci_gatt_del_include_service"),
    (0xfd0a, "aci_gatt_set_event_mask"),
    (0xfd0b, "aci_gatt_exchange_config"),
    (0xfd0c, "aci_att_find_info_req"),
    (0xfd0d, "aci_att_find_by_type_value_req"),
    (0xfd0e, "aci_att_read_by_type_req"),
    (0xfd0f, "aci_att_read_by_group_type_req"),
    (0xfd10, "aci_att_prepare_write_req"),
    (0xfd11, "aci_att_execute_write_req"),
    (0xfd12, "aci_gatt_disc_all_primary_services"),
    (0xfd13, "aci_gatt_disc_primary_service_by_uuid"),
    (0xfd14, "aci_gatt_find_included_services"),
    (0xfd15, "aci_gatt_disc_all_char_of_service"),
    (0xfd16, "aci_gatt_disc_char_by_uuid"),
    (0xfd17, "aci_gatt_disc_all_char_desc"),
    (0xfd18, "aci_gatt_read_char_value"),
    (0xfd19, "aci_gatt_read_using_char_uuid"),
    (0xfd1a, "aci_gatt_read_long_char_value"),
    (0xfd1b, "aci_gatt_read_multiple_char_value"),
    (0xfd1c, "aci_gatt_write_char_value"),
    (0xfd1d, "aci_gatt_write_long_char_value"),
    (0xfd1e, "aci_gatt_write_char_reliable"),
    (0xfd1f, "aci_gatt_write_long_char_desc"),
    (0xfd20, "aci_gatt_read_long_char_desc"),
    (0xfd21, "aci_gatt_write_char_desc"),
    (0xfd22, "aci_gatt_read_char_desc"),
    (0xfd23, "aci_gatt_write_without_resp"),
    (0xfd24, "aci_gatt_signed_write_without_resp"),
    (0xfd25, "aci_gatt_confirm_indication"),
    (0xfd26, "aci_gatt_write_resp"),
    (0xfd27, "aci_gatt_allow_read"),
    (0xfd28, "aci_gatt_set_security_permission"),
    (0xfd29, "aci_gatt_set_desc_value"),
    (0xfd2a, "aci_gatt_read_handle_value"),
    (0xfd2c, "aci_gatt_update_char_value_ext"),
    (0xfd2d, "aci_gatt_deny_read"),
    (0xfd2e, "aci_gatt_set_access_permission"),
    (0xfd81, "aci_l2cap_connection_parameter_update_req"),
    (0xfd82, "aci_l2cap_connection_parameter_update_resp"),
    (0xfd88, "aci_l2cap_coc_connect"),
    (0xfd89, "aci_l2cap_coc_connect_confirm"),
    (0xfd8a, "aci_l2cap_coc_reconf"),
    (0xfd8b, "aci_l2cap_coc_reconf_confirm"),
    (0xfd8c, "aci_l2cap_coc_disconnect"),
    (0xfd8d, "aci_l2cap_coc_flow_control"),
    (0xfd8e, "aci_l2cap_coc_tx_data"),
];

/// ACI vendor event codes and names.
pub(crate) const VENDOR_EVENTS: &[(u16, &str)] = &[
    (0x0004, "aci_hal_end_of_radio_activity"),
    (0x0005, "aci_hal_scan_req_report"),
    (0x0006, "aci_hal_fw_error"),
    (0x0400, "aci_gap_limited_discoverable"),
    (0x0401, "aci_gap_pairing_complete"),
    (0x0402, "aci_gap_pass_key_req"),
    (0x0403, "aci_gap_authorization_req"),
    (0x0404, "aci_gap_slave_security_initiated"),
    (0x0405, "aci_gap_bond_lost"),
    (0x0407, "aci_gap_proc_complete"),
    (0x0408, "aci_gap_addr_not_resolved"),
    (0x0409, "aci_gap_numeric_comparison_value"),
    (0x040a, "aci_gap_keypress_notification"),
    (0x0800, "aci_l2cap_connection_update_resp"),
    (0x0801, "aci_l2cap_proc_timeout"),
    (0x0802, "aci_l2cap_connection_update_req"),
    (0x080a, "aci_l2cap_command_reject"),
    (0x0810, "aci_l2cap_coc_connect"),
    (0x0811, "aci_l2cap_coc_connect_confirm"),
    (0x0812, "aci_l2cap_coc_reconf"),
    (0x0813, "aci_l2cap_coc_reconf_confirm"),
    (0x0814, "aci_l2cap_coc_disconnect"),
    (0x0815, "aci_l2cap_coc_flow_control"),
    (0x0816, "aci_l2cap_coc_rx_data"),
    (0x0817, "aci_l2cap_coc_tx_pool_available"),
    (0x0c01, "aci_gatt_attribute_modified"),
    (0x0c02, "aci_gatt_proc_timeout"),
    (0x0c03, "aci_att_exchange_mtu_resp"),
    (0x0c04, "aci_att_find_info_resp"),
    (0x0c05, "aci_att_find_by_type_value_resp"),
    (0x0c06, "aci_att_read_by_type_resp"),
    (0x0c07, "aci_att_read_resp"),
    (0x0c08, "aci_att_read_blob_resp"),
    (0x0c09, "aci_att_read_multiple_resp"),
    (0x0c0a, "aci_att_read_by_group_type_resp"),
    (0x0c0c, "aci_att_prepare_write_resp"),
    (0x0c0d, "aci_att_exec_write_resp"),
    (0x0c0e, "aci_gatt_indication"),
    (0x0c0f, "aci_gatt_notification"),
    (0x0c10, "aci_gatt_proc_complete"),
    (0x0c11, "aci_gatt_error_resp"),
    (0x0c12, "aci_gatt_disc_read_char_by_uuid_resp"),
    (0x0c13, "aci_gatt_write_permit_req"),
    (0x0c14, "aci_gatt_read_permit_req"),
    (0x0c15, "aci_gatt_read_multi_permit_req"),
    (0x0c16, "aci_gatt_tx_pool_available"),
    (0x0c17, "aci_gatt_server_confirmation"),
    (0x0c18, "aci_gatt_prepare_write_permit_req"),
    (0x0c1d, "aci_gatt_read_ext"),
    (0x0c1e, "aci_gatt_indication_ext"),
    (0x0c1f, "aci_gatt_notification_ext"),
];

/// Name of a command opcode (`OGF << 10 | OCF`).
pub fn command_name(opcode: u16) -> Option<&'static str> {
    COMMANDS.iter().find(|c| c.0 == opcode).map(|c| c.1)
}

fn command(opcode: u16) -> String {
    match command_name(opcode) {
        Some(n) => n.to_string(),
        None => format!("opcode {opcode:#06x}"),
    }
}

/// Name of an ACI vendor event code.
pub fn vendor_event_name(code: u16) -> Option<&'static str> {
    VENDOR_EVENTS.iter().find(|c| c.0 == code).map(|c| c.1)
}

/// Standard HCI event names.
fn event_name(code: u8) -> Option<&'static str> {
    Some(match code {
        0x05 => "disconnection complete",
        0x08 => "encryption change",
        0x0c => "read remote version complete",
        0x0e => "command complete",
        0x0f => "command status",
        0x10 => "hardware error",
        0x13 => "number of completed packets",
        0x1a => "data buffer overflow",
        0x30 => "encryption key refresh complete",
        0x3e => "LE meta",
        0xff => "vendor",
        _ => return None,
    })
}

/// LE meta event sub-event names.
fn le_subevent_name(code: u8) -> Option<&'static str> {
    Some(match code {
        0x01 => "connection complete",
        0x02 => "advertising report",
        0x03 => "connection update complete",
        0x04 => "read remote features complete",
        0x05 => "long term key request",
        0x06 => "remote connection parameter request",
        0x07 => "data length change",
        0x08 => "read local P-256 public key complete",
        0x09 => "generate DHKey complete",
        0x0a => "enhanced connection complete",
        0x0b => "directed advertising report",
        0x0c => "PHY update complete",
        _ => return None,
    })
}

/// HCI status / error code meaning (0 = success).
pub fn status(code: u8) -> String {
    let s = match code {
        0x00 => "success",
        0x01 => "unknown command",
        0x02 => "unknown connection",
        0x05 => "authentication failure",
        0x06 => "PIN or key missing",
        0x07 => "memory capacity exceeded",
        0x08 => "connection timeout",
        0x0c => "command disallowed",
        0x12 => "invalid parameters",
        0x13 => "remote user terminated connection",
        0x16 => "connection terminated by local host",
        0x1a => "unsupported remote feature",
        0x1f => "unspecified error",
        0x22 => "LL response timeout",
        0x3b => "unacceptable connection parameters",
        0x3d => "connection terminated due to MIC failure",
        0x3e => "connection failed to be established",
        0x40 => "unknown connection id",
        0x41 => "failed",
        0x42 => "invalid parameters",
        0x43 => "busy",
        0x45 => "pending",
        0x46 => "not allowed",
        0x47 => "error",
        0x48 => "out of memory",
        0x50 => "invalid CID",
        0x5c => "device not found",
        0x5d => "security database full",
        0x5e => "device not bonded",
        0x60 => "invalid handle",
        0x61 => "out of handles",
        0x62 => "invalid operation",
        0x63 => "characteristic already exists",
        0x64 => "insufficient resources",
        0x65 => "security permission error",
        0x70 => "address not resolved",
        0x82 => "no valid slot",
        0xff => "timeout",
        _ => return format!("status {code:#04x}"),
    };
    if code == 0 { s.into() } else { format!("{s} ({code:#04x})") }
}

fn le16(b: &[u8], at: usize) -> Option<u16> {
    Some(u16::from_le_bytes([*b.get(at)?, *b.get(at + 1)?]))
}

/// A BLE address (little endian on the wire) in the usual notation.
pub fn bd_addr(b: &[u8]) -> String {
    b.iter().rev().map(|x| format!("{x:02X}")).collect::<Vec<_>>().join(":")
}

/// What an HCI packet received from the radio says. `att_write` is set to
/// `(attribute handle, offset, data)` for a GATT attribute write by the
/// peer (the APDU transport's way in).
pub fn describe_packet(p: &[u8], att_write: &mut Option<(u16, u16, Vec<u8>)>) -> String {
    let Some((&kind, rest)) = p.split_first() else {
        return "empty HCI packet".into();
    };
    match kind {
        0x04 => describe_event(rest, att_write),
        0x02 => format!("HCI ACL data, {} bytes: {}", rest.len(), hex_trunc(rest, 32)),
        k => format!("HCI packet type {k:#04x}, {} bytes: {}", rest.len(), hex_trunc(rest, 32)),
    }
}

fn describe_event(p: &[u8], att_write: &mut Option<(u16, u16, Vec<u8>)>) -> String {
    let (Some(&code), Some(&len)) = (p.first(), p.get(1)) else {
        return format!("truncated HCI event: {}", hex_trunc(p, 32));
    };
    let params = &p[2.min(p.len())..];
    let mut s = match event_name(code) {
        Some(n) => format!("HCI {n}"),
        None => format!("HCI event {code:#04x}"),
    };
    if params.len() != len as usize {
        s += &format!(" (length {len}, {} bytes present)", params.len());
    }
    let more = match code {
        0x0e if params.len() >= 3 => {
            let op = le16(params, 1).unwrap_or(0);
            let mut t = format!(": {}", command(op));
            if let Some(&st) = params.get(3) {
                t += &format!(" → {}", status(st));
            }
            if params.len() > 4 {
                t += &format!(", returns {}", hex_trunc(&params[4..], 24));
            }
            t
        }
        0x0f if params.len() >= 4 => {
            let op = le16(params, 2).unwrap_or(0);
            format!(": {} → {}", command(op), status(params[0]))
        }
        0x05 if params.len() >= 4 => format!(
            ": handle {:#06x} {}, reason {}",
            le16(params, 1).unwrap_or(0),
            status(params[0]),
            status(params[3])
        ),
        0x08 if params.len() >= 4 => format!(
            ": handle {:#06x} {}, encryption {}",
            le16(params, 1).unwrap_or(0),
            status(params[0]),
            if params[3] != 0 { "on" } else { "off" }
        ),
        0x3e if !params.is_empty() => {
            let sub = params[0];
            let name = le_subevent_name(sub).map_or(format!("sub-event {sub:#04x}"), str::to_string);
            let mut t = format!(": {name}");
            if matches!(sub, 0x01 | 0x0a) && params.len() >= 11 {
                t += &format!(
                    " {}, handle {:#06x}, {} peer {}",
                    status(params[1]),
                    le16(params, 2).unwrap_or(0),
                    if params[4] == 0 { "central" } else { "peripheral" },
                    bd_addr(&params[6..12.min(params.len())])
                );
            } else if params.len() > 1 {
                t += &format!(" {}", hex_trunc(&params[1..], 24));
            }
            t
        }
        0xff if params.len() >= 2 => {
            let ecode = le16(params, 0).unwrap_or(0);
            let body = &params[2..];
            let name = vendor_event_name(ecode).map_or(format!("{ecode:#06x}"), str::to_string);
            let mut t = format!(" {name}");
            // aci_gatt_attribute_modified: connection, attribute, offset,
            // length, data.
            if ecode == 0x0c01 && body.len() >= 8 {
                let attr = le16(body, 2).unwrap_or(0);
                let off = le16(body, 4).unwrap_or(0);
                let n = (le16(body, 6).unwrap_or(0) as usize).min(body.len() - 8);
                let data = body[8..8 + n].to_vec();
                t += &format!(": attribute {attr:#06x} offset {off}, {n} bytes: {}", hex_trunc(&data, 24));
                *att_write = Some((attr, off, data));
            } else if !body.is_empty() {
                t += &format!(": {}", hex_trunc(body, 24));
            }
            t
        }
        _ if !params.is_empty() => format!(": {}", hex_trunc(params, 24)),
        _ => String::new(),
    };
    s + &more
}

/// What an HCI command sent to the radio does (`p` = opcode, big endian as
/// SEPROXYHAL sends it, then the parameters). `notify` is set to
/// `(characteristic handle, value)` for a characteristic value update (the
/// APDU transport's way out).
pub fn describe_command(p: &[u8], notify: &mut Option<(u16, Vec<u8>)>) -> String {
    if p.len() < 2 {
        return format!("truncated HCI command: {}", hex_trunc(p, 32));
    }
    let op = u16::from_be_bytes([p[0], p[1]]);
    let params = &p[2..];
    let mut s = format!("HCI command {}", command(op));
    match op {
        // aci_gatt_update_char_value: service, characteristic, offset,
        // length, value.
        0xfd06 if params.len() >= 6 => {
            let ch = le16(params, 2).unwrap_or(0);
            let n = (params[5] as usize).min(params.len() - 6);
            let value = params[6..6 + n].to_vec();
            s += &format!(
                ": characteristic {ch:#06x} offset {}, {n} bytes: {}",
                params[4],
                hex_trunc(&value, 24)
            );
            *notify = Some((ch, value));
        }
        // aci_hal_write_config_data: offset, length, value.
        0xfc0c if params.len() >= 2 => {
            let what = match params[0] {
                0x00 => "public address",
                0x08 => "ER key",
                0x18 => "IR key",
                0x2e => "random static address",
                _ => "offset",
            };
            let v = &params[2..];
            let val = if matches!(params[0], 0x00 | 0x2e) {
                bd_addr(v)
            } else {
                hex_trunc(v, 24)
            };
            s += &format!(": {what} ({:#04x}) = {val}", params[0]);
        }
        // hci_le_set_scan_response_data / advertising data: AD structures.
        0x2008 | 0x2009 if !params.is_empty() => {
            s += &format!(": {}", advertising_data(&params[1..1 + (params[0] as usize).min(params.len() - 1)]));
        }
        _ if !params.is_empty() => s += &format!(": {}", hex_trunc(params, 24)),
        _ => {}
    }
    s
}

/// Advertising / scan response AD structures (length, type, data).
fn advertising_data(mut b: &[u8]) -> String {
    let mut parts = Vec::new();
    while let Some((&len, rest)) = b.split_first() {
        if len == 0 || rest.len() < len as usize {
            break;
        }
        let (ty, data) = (rest[0], &rest[1..len as usize]);
        parts.push(match ty {
            0x01 => format!("flags {:#04x}", data.first().unwrap_or(&0)),
            0x08 | 0x09 => format!("name \"{}\"", String::from_utf8_lossy(data)),
            0x12 if data.len() == 4 => format!(
                "connection interval {}..{} ×1.25 ms",
                u16::from_le_bytes([data[0], data[1]]),
                u16::from_le_bytes([data[2], data[3]])
            ),
            _ => format!("AD {ty:#04x} {}", hex_trunc(data, 16)),
        });
        b = &rest[len as usize..];
    }
    if parts.is_empty() {
        "no AD structures".into()
    } else {
        parts.join(", ")
    }
}
